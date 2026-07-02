#include "board/rid_led.h"

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol/rid_state.h"

static const char *TAG = "rid_led";

#if SERY_RID_STATUS_LED_GPIO >= 0
static StackType_t s_led_stack[1024];
static StaticTask_t s_led_tcb;

static void led_task(void *arg) {
    (void)arg;
    bool level = false;
    uint32_t last_toggle_ms = 0;

    while (true) {
        char reason[64] = {0};
        bool ok = rid_state_arm_status(reason, sizeof(reason));
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (ok) {
            gpio_set_level(SERY_RID_STATUS_LED_GPIO, SERY_RID_STATUS_LED_ON_LEVEL);
            level = true;
        } else if (now - last_toggle_ms >= 100) {
            last_toggle_ms = now;
            level = !level;
            gpio_set_level(SERY_RID_STATUS_LED_GPIO,
                           level ? SERY_RID_STATUS_LED_ON_LEVEL : !SERY_RID_STATUS_LED_ON_LEVEL);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

esp_err_t rid_led_start(void) {
#if SERY_RID_STATUS_LED_GPIO >= 0
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << SERY_RID_STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    xTaskCreateStatic(led_task, "rid_led", 1024, NULL, 2, s_led_stack, &s_led_tcb);
    ESP_LOGI(TAG, "status LED enabled on GPIO%d", SERY_RID_STATUS_LED_GPIO);
#else
    ESP_LOGI(TAG, "status LED disabled");
#endif
    return ESP_OK;
}
