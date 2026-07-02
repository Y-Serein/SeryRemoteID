#include "net/rid_wifi.h"

#include <limits.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "opendroneid.h"
#include "protocol/rid_state.h"

static const char *TAG = "rid_wifi";

static StackType_t s_wifi_stack[SERY_RID_TASK_STACK_WORDS];
static StaticTask_t s_wifi_tcb;
static uint8_t s_wifi_mac[6];
static uint8_t s_nan_counter;
static uint8_t s_beacon_counter;

static uint8_t dbm_to_quarter_dbm(float dbm) {
    if (dbm < 2.0f) {
        dbm = 2.0f;
    }
    if (dbm > 20.0f) {
        dbm = 20.0f;
    }
    return (uint8_t)((dbm + 1.125f) * 4.0f);
}

static void generate_local_mac(uint8_t mac[6]) {
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    memcpy(mac, &r1, 4);
    memcpy(&mac[4], &r2, 2);
    mac[0] |= 0x02;
    mac[0] &= 0xFE;
}

static int interval_ms(float hz) {
    if (hz <= 0.0f) {
        return INT_MAX;
    }
    int interval = (int)(1000.0f / hz);
    return interval < 1 ? 1 : interval;
}

static bool transmit_nan(ODID_UAS_Data *uas) {
    uint8_t buffer[1024] = {0};

    int len = odid_wifi_build_nan_sync_beacon_frame((char *)s_wifi_mac,
                                                    buffer,
                                                    sizeof(buffer));
    if (len > 0 &&
        esp_wifi_80211_tx(WIFI_IF_AP, buffer, len, true) != ESP_OK) {
        return false;
    }

    len = odid_wifi_build_message_pack_nan_action_frame(uas,
                                                        (char *)s_wifi_mac,
                                                        ++s_nan_counter,
                                                        buffer,
                                                        sizeof(buffer));
    if (len > 0 &&
        esp_wifi_80211_tx(WIFI_IF_AP, buffer, len, true) != ESP_OK) {
        return false;
    }

    return len > 0;
}

static bool update_beacon_vendor_ie(ODID_UAS_Data *uas) {
    const rid_config_t *cfg = cfg_get();
    uint8_t buffer[1024] = {0};
    const char dummy_ssid[] = "UAS_ID_OPEN";
    uint16_t interval_tu = (uint16_t)interval_ms(cfg->wifi_beacon_rate_hz);

    int len = odid_wifi_build_message_pack_beacon_frame(uas,
                                                        (char *)s_wifi_mac,
                                                        dummy_ssid,
                                                        strlen(dummy_ssid),
                                                        interval_tu,
                                                        ++s_beacon_counter,
                                                        buffer,
                                                        sizeof(buffer));
    if (len <= 0) {
        return false;
    }

    const size_t header_offset = 58;
    if ((size_t)len <= header_offset) {
        return false;
    }

    size_t payload_len = (size_t)len - header_offset;
    if (payload_len + 4 > 255) {
        return false;
    }

    uint8_t ie_buf[sizeof(vendor_ie_data_t) + 255] = {0};
    vendor_ie_data_t *ie = (vendor_ie_data_t *)ie_buf;
    ie->element_id = WIFI_VENDOR_IE_ELEMENT_ID;
    ie->length = (uint8_t)(payload_len + 4);
    ie->vendor_oui[0] = 0xFA;
    ie->vendor_oui[1] = 0x0B;
    ie->vendor_oui[2] = 0xBC;
    ie->vendor_oui_type = 0x0D;
    memcpy(ie->payload, &buffer[header_offset], payload_len);

    (void)esp_wifi_set_vendor_ie(false, WIFI_VND_IE_TYPE_BEACON, WIFI_VND_IE_ID_0, ie);
    (void)esp_wifi_set_vendor_ie(false, WIFI_VND_IE_TYPE_PROBE_RESP, WIFI_VND_IE_ID_0, ie);

    if (esp_wifi_set_vendor_ie(true, WIFI_VND_IE_TYPE_BEACON, WIFI_VND_IE_ID_0, ie) != ESP_OK) {
        return false;
    }
    if (esp_wifi_set_vendor_ie(true, WIFI_VND_IE_TYPE_PROBE_RESP, WIFI_VND_IE_ID_0, ie) != ESP_OK) {
        return false;
    }

    return true;
}

static void wifi_broadcast_task(void *arg) {
    (void)arg;
    uint32_t last_nan_ms = 0;
    uint32_t last_beacon_ms = 0;
    uint32_t last_skip_log_ms = 0;

    ESP_LOGI(TAG, "WiFi broadcast task started");

    while (true) {
        const rid_config_t *cfg = cfg_get();
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ODID_UAS_Data uas;
        char reason[128] = {0};

        if (!rid_state_build_uas_data(&uas, reason, sizeof(reason))) {
            if (now - last_skip_log_ms > 2000) {
                last_skip_log_ms = now;
                ESP_LOGW(TAG, "skip RemoteID broadcast: %s", reason[0] ? reason : "no valid data");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int nan_interval = interval_ms(cfg->wifi_nan_rate_hz);
        if (cfg->wifi_nan_rate_hz > 0.0f && now - last_nan_ms >= (uint32_t)nan_interval) {
            last_nan_ms = now;
            if (!transmit_nan(&uas)) {
                ESP_LOGW(TAG, "NAN transmit failed");
            }
        }

        int beacon_interval = interval_ms(cfg->wifi_beacon_rate_hz);
        if (cfg->wifi_beacon_rate_hz > 0.0f &&
            now - last_beacon_ms >= (uint32_t)beacon_interval) {
            last_beacon_ms = now;
            if (!update_beacon_vendor_ie(&uas)) {
                ESP_LOGW(TAG, "Beacon vendor IE update failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t rid_wifi_start(void) {
    const rid_config_t *cfg = cfg_get();

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "esp_wifi_init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "esp_wifi_set_storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "esp_wifi_set_mode failed");

    generate_local_mac(s_wifi_mac);
    ESP_RETURN_ON_ERROR(esp_wifi_set_mac(WIFI_IF_AP, s_wifi_mac), TAG, "esp_wifi_set_mac failed");

    wifi_config_t ap_config = {0};
    strncpy((char *)ap_config.ap.ssid, cfg->wifi_ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen((const char *)ap_config.ap.ssid);
    ap_config.ap.channel = cfg->wifi_channel;
    ap_config.ap.max_connection = 1;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ap_config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config),
                        TAG,
                        "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(dbm_to_quarter_dbm(cfg->wifi_power_dbm)),
                        TAG,
                        "esp_wifi_set_max_tx_power failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20),
                        TAG,
                        "esp_wifi_set_bandwidth failed");

    xTaskCreateStatic(wifi_broadcast_task,
                      "rid_wifi",
                      SERY_RID_TASK_STACK_WORDS,
                      NULL,
                      4,
                      s_wifi_stack,
                      &s_wifi_tcb);

    ESP_LOGI(TAG,
             "WiFi AP started: ssid='%s' ch=%u mac=%02X:%02X:%02X:%02X:%02X:%02X",
             cfg->wifi_ssid,
             (unsigned)cfg->wifi_channel,
             s_wifi_mac[0],
             s_wifi_mac[1],
             s_wifi_mac[2],
             s_wifi_mac[3],
             s_wifi_mac[4],
             s_wifi_mac[5]);
    return ESP_OK;
}
