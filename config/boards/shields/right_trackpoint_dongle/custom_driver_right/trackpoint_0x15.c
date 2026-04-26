/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * 优化说明:
 * 1. 彻底移除引发卡顿的 k_sleep(40)。引入“余数累加器 (Residue Accumulator)”实现平滑处理。
 * 2. 移除消耗极大的 5ms 轮询任务，改为低功耗硬件 GPIO 中断 (GPIO Interrupt) 驱动。
 * 3. 修复编译错误：在 C 代码中直接构造 gpio_dt_spec。
 * 4. [新增] 引入防误触的双轴滚动独立锁定算法 (Dominant Axis Lock)，确保单次只触发一个方向滚动。
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

/* ========= Motion GPIO (手工构造，绕过 YAML 限制) ========= */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)
#define SPANCE_POSITION_CODE 62 
/* ========= 全局状态 ========= */
static bool space_pressed = false;

/* ========= Space 按键监听 ========= */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) return 0;

    if (ev->position == SPANCE_POSITION_CODE) { // Space position code
        space_pressed = ev->state;
    }
    return 0;
}
ZMK_LISTENER(trackpoint_space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_space_listener, zmk_position_state_changed);

/* ========= TrackPoint 结构 ========= */
struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    uint32_t last_packet_time;
    int16_t scroll_residue_x; // 横向滚轮累加器
    int16_t scroll_residue_y; // 纵向滚轮累加器
};

/* ========= 指数加速计算 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_MAX_MULT 2.0f
static inline float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0) delta_ms = 1;

    // 优化1: abs 用于整型比 fabsf 快
    int dist = abs(dx) + abs(dy);
    if (dist < 1) return 1.0f;

    float speed = (float)dist / (float)delta_ms;
    
    // 优化2: 移除昂贵的 powf(1.04f, speed / 0.03f)。
    float mult = expf(speed * 1.307357f);

    return (mult > TP_MAX_MULT) ? TP_MAX_MULT : mult;
}
#endif

/* ========= 读取数据包 ========= */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};

    int ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);
    if (ret < 0) return ret;

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0) return -EIO;

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* ========= 工作队列处理 ========= */
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    // 当引脚处于有效状态 (Active Low，0代表有数据) 时循环读取
    while (gpio_pin_get_dt(&cfg->motion_gpio) > 0) {
        int8_t dx = 0, dy = 0;

        if (trackpoint_read_packet(dev, &dx, &dy) == 0) {
            uint32_t now = k_uptime_get_32();

            if (!space_pressed) {
                /* ========================================================= */
                /* 1. 判定主导方向 (Dominant Axis) 以实现单向锁定防误触      */
                /* ========================================================= */
                int abs_dx = abs(dx);
                int abs_dy = abs(dy);

                // 使用整数比例判定：某个方向的力度必须是另一方向的 1.5 倍以上才触发
                // (即: abs_a > abs_b * 1.5 === abs_a * 2 > abs_b * 3)
                if (abs_dy * 2 > abs_dx * 3) {
                    dx = 0; // 锁定为纯竖向
                } else if (abs_dx * 2 > abs_dy * 3) {
                    dy = 0; // 锁定为纯横向
                } else {
                    // 处于对角线模糊死区，丢弃操作防止乱滚
                    dx = 0;
                    dy = 0; 
                }

                /* ========================================================= */
                /* 2. 处理竖向滚动 (Y 轴)                                    */
                /* ========================================================= */
                if (dy != 0) {
                    data->scroll_residue_y += dy;
                    int divisor = 24; // 基础平滑系数
                    if (abs_dy >= 128) divisor = 6;
                    else if (abs_dy >= 64) divisor = 10;
                    else if (abs_dy >= 32) divisor = 14;
                    else if (abs_dy >= 21) divisor = 18;
                    else if (abs_dy < 3) divisor = 100;

                    int16_t scroll_ticks = data->scroll_residue_y / divisor;
                    if (scroll_ticks != 0) {
                        input_report_rel(dev, INPUT_REL_WHEEL, scroll_ticks, true, K_FOREVER);
                        data->scroll_residue_y %= divisor;
                    }
                } else {
                    data->scroll_residue_y = 0; // 无该轴操作时清零残余，彻底隔绝积累
                }

                /* ========================================================= */
                /* 3. 处理横向滚动 (X 轴)                                    */
                /* ========================================================= */
                if (dx != 0) {
                    data->scroll_residue_x += dx;
                    int divisor = 24;
                    if (abs_dx >= 128) divisor = 6;
                    else if (abs_dx >= 64) divisor = 10;
                    else if (abs_dx >= 32) divisor = 14;
                    else if (abs_dx >= 21) divisor = 18;
                    else if (abs_dx < 3) divisor = 100;

                    int16_t scroll_ticks = data->scroll_residue_x / divisor;
                    if (scroll_ticks != 0) {
                        input_report_rel(dev, INPUT_REL_HWHEEL, -scroll_ticks, true, K_FOREVER);
                        data->scroll_residue_x %= divisor;
                    }
                } else {
                    data->scroll_residue_x = 0; // 无该轴操作时清零残余
                }

            } else {
                /* 正常鼠标移动 */
                uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
                float tp_factor = 0.3f + 0.01f * tp_led_brt;

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
                uint32_t delta = now - data->last_packet_time;
                float exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
                float exp_mult = 1.0f;
#endif
                float fx = dx * 0.5f * tp_factor * exp_mult;
                float fy = dy * 0.5f * tp_factor * exp_mult;

                input_report_rel(dev, INPUT_REL_X, -(int)fx, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, -(int)fy, true, K_FOREVER);
            }
            data->last_packet_time = now;
        } else {
            break; // I2C 出错，跳出循环
        }
    }
}

/* ========= 硬件中断 ISR ========= */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);
    k_work_submit(&data->work);
}

/* ========= 初始化函数 ========= */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio)) return -ENODEV;

    data->dev = dev;
    data->scroll_residue_x = 0;
    data->scroll_residue_y = 0;
    data->last_packet_time = k_uptime_get_32();
    k_work_init(&data->work, trackpoint_work_cb);

    // 配置硬件引脚中断
    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    LOG_INF("TrackPoint Driver Initialized with Interrupts");
    return 0;
}

/* ★ 修改点：用手动赋值替换 GPIO_DT_SPEC_INST_GET，绕过 YAML 报错 */
#define TRACKPOINT_DEFINE(inst)                                                \
    static struct trackpoint_data trackpoint_data_##inst;                      \
    static const struct trackpoint_config trackpoint_config_##inst = {         \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                     \
        .motion_gpio = {                                                       \
            .port = DEVICE_DT_GET(MOTION_GPIO_NODE),                           \
            .pin = MOTION_GPIO_PIN,                                            \
            .dt_flags = MOTION_GPIO_FLAGS                                      \
        },                                                                     \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,\
                          &trackpoint_config_##inst, POST_KERNEL,              \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);