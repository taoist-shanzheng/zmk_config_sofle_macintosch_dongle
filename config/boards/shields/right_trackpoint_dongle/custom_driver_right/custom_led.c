/*
 * custom_led_backlight_follow.c
 * 优化说明 (v4 - 纯事件驱动版):
 * 1. 彻底消灭了“轮询(Polling)”的概念。
 * 2. 仅在特定亮度键按下时，触发一次单次的更新任务 (Update Work)。
 * 3. 完美结合了 ZMK 的底层背光机制，不仅 0 功耗，而且代码逻辑极其精简。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/math_extras.h>

#include <zmk/backlight.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include "custom_led.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_custom_led),
             "Custom LED enabled but no zmk,custom_led chosen node found");

static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_custom_led));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define LED_NUM (DT_NUM_CHILD(DT_CHOSEN(zmk_custom_led)))

#define BRT_MIN 10
#define OFF_DELAY_MS 3000
#define FADE_STEP_MS 20
#define FADE_STEPS 20

static struct k_work_delayable auto_off_work;
static struct k_work_delayable update_work; // 替代了之前的 poll_work
static struct k_work_delayable fade_work;

static uint8_t last_brt = 255;
static uint8_t current_brt = 0;
static uint8_t start_brt = 0;
static uint8_t target_brt = 0;
static int fade_step = -1;

/* 对外状态：最近一次有效亮度，这确保了灯灭后小红点灵敏度不变 */
static uint8_t last_valid_brt = BRT_MIN;

/* 定义亮度调节键的物理位置 (Position) */
#define BRT_KEY_UP_POS 49
#define BRT_KEY_DOWN_POS 50

/* === ZMK Event Listener: 单次按键触发 === */
static int led_activity_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) return 0;

    // 当按下指定的亮度调节键时
    if (ev->state && (ev->position == BRT_KEY_UP_POS || ev->position == BRT_KEY_DOWN_POS)) {
        // 延迟 50ms 执行更新，确保 ZMK 的核心 backlight 模块已经处理完这次按键并更新了内部亮度值
        k_work_reschedule(&update_work, K_MSEC(50));
    }
    return 0;
}
ZMK_LISTENER(custom_led_activity_listener, led_activity_listener);
ZMK_SUBSCRIPTION(custom_led_activity_listener, zmk_position_state_changed);

/* === Immediate apply LED === */
static void apply_led(uint8_t brightness) {
    if (!device_is_ready(led_dev))
        return;

    for (int i = 0; i < LED_NUM; i++) {
        led_set_brightness(led_dev, i, brightness);
    }
    current_brt = brightness;
}

/* === Fade animation handler === */
static void fade_handler(struct k_work *work) {
    if (fade_step < 0) return;

    int new_level = start_brt + ((target_brt - start_brt) * fade_step) / FADE_STEPS;
    apply_led((uint8_t)new_level);

    fade_step++;
    if (fade_step > FADE_STEPS) {
        apply_led(target_brt);
        fade_step = -1;
        return;
    }

    k_work_reschedule(&fade_work, K_MSEC(FADE_STEP_MS));
}

/* === Start fade === */
static void fade_to(uint8_t new_brt) {
    target_brt = new_brt;
    start_brt = current_brt;
    fade_step = 0;
    k_work_reschedule(&fade_work, K_MSEC(FADE_STEP_MS));
}

/* === Auto-off timeout → fade out === */
static void auto_off_handler(struct k_work *work) {
    if (current_brt > 0) {
        LOG_INF("Idle timeout -> fade out to 0");
        fade_to(0);
    }
}

/* === 单次更新亮度处理 (不再循环) === */
static void update_brightness_handler(struct k_work *work) {
    uint8_t brt = zmk_backlight_get_brt();

    if (brt != last_brt) {
        last_brt = brt;
        uint8_t led_level = (brt == 0) ? 0 : MAX(BRT_MIN, brt);

        // 灯亮着时，才更新有效亮度（保护小红点灵敏度）
        if (led_level > 0) {
            last_valid_brt = led_level;
        }

        if (led_level == 0) {
            k_work_reschedule(&auto_off_work, K_MSEC(OFF_DELAY_MS));
        } else {
            if (current_brt == 0) {
                LOG_INF("Fade-in from dark → %d", led_level);
                fade_to(led_level);
            } else {
                apply_led(led_level);
            }
            k_work_reschedule(&auto_off_work, K_MSEC(OFF_DELAY_MS));
        }
    } else {
        // 如果亮度已达到最大或最小值没有变化，但用户按了调节键，
        // 我们依然重新刷新 3 秒灯光存活时间，并确保灯是被唤醒的状态
        if (brt > 0) {
            if (current_brt == 0) fade_to(MAX(BRT_MIN, brt));
            k_work_reschedule(&auto_off_work, K_MSEC(OFF_DELAY_MS));
        }
    }
    
}

/* === Public API === */
uint8_t custom_led_get_last_valid_brightness(void) { return last_valid_brt; }

/* === Init === */
static int init_led_follow(void) {
    if (!device_is_ready(led_dev)) return -ENODEV;

    k_work_init_delayable(&auto_off_work, auto_off_handler);
    k_work_init_delayable(&update_work, update_brightness_handler);
    k_work_init_delayable(&fade_work, fade_handler);

    uint8_t boot = zmk_backlight_get_brt();
    uint8_t led_level = (boot == 0) ? 0 : MAX(BRT_MIN, boot);

    if (led_level > 0) {
        last_valid_brt = led_level;
    }
    apply_led(led_level);

    k_work_reschedule(&auto_off_work, K_MSEC(OFF_DELAY_MS));

    LOG_INF("LED driver pure event-driven initialized");
    return 0;
}

SYS_INIT(init_led_follow, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);