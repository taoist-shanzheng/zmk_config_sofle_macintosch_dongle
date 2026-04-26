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

/* ========================================================================= */
/* 鼠标与滚轮可调参数 (用户配置区)                         */
/* ========================================================================= */

// --- 滚轮方向配置 ---
// 根据你的习惯反转滚轮方向 (1 为默认正向, -1 为反向)
#define SCROLL_X_DIR 1
#define SCROLL_Y_DIR 1

// --- 滚轮灵敏度与粒度配置 (线性映射算法) ---
// 过滤轻微手抖的死区 (小于此值的物理移动将被忽略)
#define SCROLL_DEADZONE 2

// TrackPoint 的最大物理输出值 (通常为 128)
#define SCROLL_INPUT_MAX 128

// 最慢滚动时的除数：决定了微操时的“细粒度”。值越大，轻轻推时滚动越慢、越细腻。
// 如果觉得现在的最小滚动还是太快，可以将此值调大 (例如 80 或 100)
#define SCROLL_DIVISOR_SLOW 60

// 最快滚动时的除数：决定了用力推时的极限滚动速度。值越小，全速滚动越快。
#define SCROLL_DIVISOR_FAST 8

// --- 防误触锁定比例配置 ---
// 判断是否为单向移动的比例 (分子 / 分母)。
// 默认为 3/2 (1.5倍)，意味着一个方向的力必须是另一个方向的 1.5 倍才触发，避免对角线乱滚。
#define DOMINANT_NUMERATOR 3
#define DOMINANT_DENOMINATOR 2

// --- 鼠标指针基础配置 ---
#define MOUSE_BASE_SPEED 0.5f
#define MOUSE_SENS_BASE 0.3f
#define MOUSE_SENS_STEP 0.01f

/* ========================================================================= */

/* ========= TrackPoint 硬件常量 ========= */
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)
// 按键位置
#define SPANCE_POSITION_CODE 62 

/* ========= 全局状态 ========= */
static bool space_pressed = false;

/* ========= Space 按键监听 ========= */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) return 0;

    if (ev->position == SPANCE_POSITION_CODE) { 
        space_pressed = ev->state;
    }
    return 0;
}
ZMK_LISTENER(trackpoint_space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_space_listener, zmk_position_state_changed);

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

    int dist = abs(dx) + abs(dy);
    if (dist < 1) return 1.0f;

    float speed = (float)dist / (float)delta_ms;
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

/* ========= ★ 抽象复用：滚轮单轴处理函数 ========= */
static inline void process_scroll_axis(const struct device *dev, int8_t delta, int16_t *residue, uint16_t input_code, int8_t dir_mult) {
    int abs_delta = abs(delta);

    // 1. 死区与空闲处理：如果没动，清空累加器防止残余位移导致之后的一格漂移
    if (abs_delta <= SCROLL_DEADZONE) {
        *residue = 0; 
        return;
    }

    // 钳制最大输入值，确保映射不会越界
    if (abs_delta > SCROLL_INPUT_MAX) {
        abs_delta = SCROLL_INPUT_MAX;
    }

    // 2. 无极连续映射 (Linear Mapping)
    // 根据手指当前力度，在 SLOW 和 FAST 之间求出一条平滑直线的当前点。
    // 这取代了之前跳跃的 if/else 阶梯，使得微操粒度无限顺滑。
    int divisor = SCROLL_DIVISOR_SLOW - 
                  ((SCROLL_DIVISOR_SLOW - SCROLL_DIVISOR_FAST) * abs_delta) / SCROLL_INPUT_MAX;

    // 安全兜底，除数不能为 0
    if (divisor < 1) divisor = 1;

    // 3. 应用乘数反转方向，并倒进累加器
    *residue += (delta * dir_mult);

    // 4. 触发滚动输出，并保留除不尽的余数
    int16_t scroll_ticks = *residue / divisor;
    if (scroll_ticks != 0) {
        input_report_rel(dev, input_code, scroll_ticks, true, K_FOREVER);
        *residue %= divisor;
    }
}

/* ========= 工作队列处理 ========= */
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    while (gpio_pin_get_dt(&cfg->motion_gpio) > 0) {
        int8_t dx = 0, dy = 0;

        if (trackpoint_read_packet(dev, &dx, &dy) == 0) {
            uint32_t now = k_uptime_get_32();

            if (!space_pressed) {
                /* 1. 主导轴锁定防误触 */
                int abs_dx = abs(dx);
                int abs_dy = abs(dy);

                if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
                    dx = 0; // 纯竖向滚动
                } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
                    dy = 0; // 纯横向滚动
                } else {
                    dx = 0; // 对角线死区，双轴丢弃
                    dy = 0; 
                }

                process_scroll_axis(dev, dx, &data->scroll_residue_x, INPUT_REL_HWHEEL, SCROLL_X_DIR);
                process_scroll_axis(dev, dy, &data->scroll_residue_y, INPUT_REL_WHEEL, SCROLL_Y_DIR);

            } else {
                uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
                float tp_factor = MOUSE_SENS_BASE + MOUSE_SENS_STEP * tp_led_brt;

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
                uint32_t delta = now - data->last_packet_time;
                float exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
                float exp_mult = 1.0f;
#endif
                float fx = dx * MOUSE_BASE_SPEED * tp_factor * exp_mult;
                float fy = dy * MOUSE_BASE_SPEED * tp_factor * exp_mult;

                input_report_rel(dev, INPUT_REL_X, -(int)fx, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, -(int)fy, true, K_FOREVER);
            }
            data->last_packet_time = now;
        } else {
            break; 
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

    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    LOG_INF("TrackPoint Driver Initialized with Interrupts");
    return 0;
}

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