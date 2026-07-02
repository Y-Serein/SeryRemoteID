#include "cfg.h"
#include "board/rid_led.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "net/rid_ble.h"
#include "net/rid_wifi.h"
#include "nvs_flash.h"
#include "protocol/rid_state.h"
#include "transport/rid_dronecan.h"
#include "transport/rid_mavlink.h"
#include "web/rid_web.h"

static const char *TAG = "app";

static esp_err_t init_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void) {
    ESP_ERROR_CHECK(init_nvs());
    cfg_init();
    rid_state_init();

    ESP_ERROR_CHECK(rid_wifi_start());
    ESP_ERROR_CHECK(rid_web_start());
    ESP_ERROR_CHECK(rid_ble_start());
    ESP_ERROR_CHECK(rid_led_start());
    ESP_ERROR_CHECK(rid_dronecan_start());
    ESP_ERROR_CHECK(rid_mavlink_start());

    ESP_LOGI(TAG, "SeryRemoteID started");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
