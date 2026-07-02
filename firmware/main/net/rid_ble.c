#include "net/rid_ble.h"

#include <stdio.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "opendroneid.h"
#include "protocol/rid_state.h"
#include "sdkconfig.h"

static const char *TAG = "rid_ble";

#if CONFIG_BT_ENABLED && CONFIG_BT_BLUEDROID_ENABLED

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

static StackType_t s_ble_stack[SERY_RID_TASK_STACK_WORDS];
static StaticTask_t s_ble_tcb;
static StaticSemaphore_t s_gap_sem_buf;
static SemaphoreHandle_t s_gap_sem;
static volatile bool s_gap_waiting;
static volatile esp_gap_ble_cb_event_t s_gap_expected_event;
static volatile int s_gap_expected_instance = -1;
static volatile esp_bt_status_t s_gap_status;
static bool s_adv_configured[2];
static bool s_adv_started[2];
static uint8_t s_msg_counters[8];
static uint8_t s_legacy_phase;

#define BLE_GAP_WAIT_TIMEOUT_MS 1000

static uint32_t adv_interval(float rate_hz, uint8_t states) {
    if (rate_hz <= 0.0f) {
        return 0;
    }
    float per_state_hz = rate_hz * (float)(states == 0 ? 1 : states);
    uint32_t interval = (uint32_t)(1000.0f / per_state_hz);
    return interval < 1 ? 1 : interval;
}

static uint32_t adv_interval_units(float rate_hz, uint8_t states) {
    uint32_t ms = adv_interval(rate_hz, states);
    if (ms == 0) {
        ms = 1000;
    }
    uint32_t units = (uint32_t)((float)ms / 0.625f);
    if (units < 0x20) {
        units = 0x20;
    }
    if (units > 0xFFFFFF) {
        units = 0xFFFFFF;
    }
    return units;
}

static int8_t clamp_power(float dbm) {
    if (dbm < -27.0f) {
        dbm = -27.0f;
    }
    if (dbm > 18.0f) {
        dbm = 18.0f;
    }
    return (int8_t)dbm;
}

static bool gap_event_info(esp_gap_ble_cb_event_t event,
                           esp_ble_gap_cb_param_t *param,
                           esp_bt_status_t *status,
                           int *instance) {
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
        *status = param->ext_adv_set_rand_addr.status;
        *instance = param->ext_adv_set_rand_addr.instance;
        return true;
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        *status = param->ext_adv_set_params.status;
        *instance = param->ext_adv_set_params.instance;
        return true;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        *status = param->ext_adv_data_set.status;
        *instance = param->ext_adv_data_set.instance;
        return true;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        *status = param->ext_adv_start.status;
        *instance = param->ext_adv_start.instance_num > 0 ? param->ext_adv_start.instance[0] : -1;
        return true;
    default:
        return false;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    esp_bt_status_t status = ESP_BT_STATUS_SUCCESS;
    int instance = -1;

    if (!s_gap_waiting || event != s_gap_expected_event ||
        !gap_event_info(event, param, &status, &instance)) {
        return;
    }

    if (s_gap_expected_instance >= 0 && instance != s_gap_expected_instance) {
        return;
    }

    s_gap_status = status;
    if (s_gap_sem != NULL) {
        xSemaphoreGive(s_gap_sem);
    }
}

static void gap_prepare_wait(esp_gap_ble_cb_event_t event, int instance) {
    while (s_gap_sem != NULL && xSemaphoreTake(s_gap_sem, 0) == pdTRUE) {
    }
    s_gap_expected_event = event;
    s_gap_expected_instance = instance;
    s_gap_status = ESP_BT_STATUS_FAIL;
    s_gap_waiting = true;
}

static esp_err_t gap_finish_wait(const char *label, uint8_t instance) {
    if (xSemaphoreTake(s_gap_sem, pdMS_TO_TICKS(BLE_GAP_WAIT_TIMEOUT_MS)) != pdTRUE) {
        s_gap_waiting = false;
        ESP_LOGW(TAG, "BLE adv set %u %s timeout", (unsigned)instance, label);
        return ESP_ERR_TIMEOUT;
    }

    s_gap_waiting = false;
    if (s_gap_status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "BLE adv set %u %s failed: gap status=%u",
                 (unsigned)instance,
                 label,
                 (unsigned)s_gap_status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t gap_call_wait(uint8_t instance,
                               const char *label,
                               esp_err_t submit_err) {
    if (submit_err != ESP_OK) {
        s_gap_waiting = false;
        ESP_LOGW(TAG, "BLE adv set %u %s submit failed: %s",
                 (unsigned)instance,
                 label,
                 esp_err_to_name(submit_err));
        return submit_err;
    }
    return gap_finish_wait(label, instance);
}

static bool configure_adv_set(uint8_t instance,
                              const esp_ble_gap_ext_adv_params_t *params,
                              esp_bd_addr_t mac) {
    gap_prepare_wait(ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT, instance);
    esp_err_t err = gap_call_wait(instance,
                                  "params",
                                  esp_ble_gap_ext_adv_set_params(instance, params));
    if (err != ESP_OK) {
        return false;
    }

    gap_prepare_wait(ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT, instance);
    err = gap_call_wait(instance,
                        "random address",
                        esp_ble_gap_ext_adv_set_rand_addr(instance, mac));
    if (err != ESP_OK) {
        return false;
    }

    return true;
}

static esp_err_t configure_adv_sets(void) {
    const rid_config_t *cfg = cfg_get();
    esp_bd_addr_t legacy_mac = {0};
    esp_bd_addr_t coded_mac = {0};
    esp_err_t err = esp_ble_gap_addr_create_static(legacy_mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE legacy random address create failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_ble_gap_addr_create_static(coded_mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE coded random address create failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_ble_gap_ext_adv_params_t legacy = {
        .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_NONCONN,
        .interval_min = (uint32_t)(0.75f * adv_interval_units(cfg->bt4_rate_hz, 7)),
        .interval_max = adv_interval_units(cfg->bt4_rate_hz, 7),
        .channel_map = ADV_CHNL_ALL,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST,
        .tx_power = clamp_power(cfg->bt4_power_dbm),
        .primary_phy = ESP_BLE_GAP_PHY_1M,
        .max_skip = 0,
        .secondary_phy = ESP_BLE_GAP_PHY_1M,
        .sid = 0,
        .scan_req_notif = false,
    };

    esp_ble_gap_ext_adv_params_t coded = {
        .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
        .interval_min = (uint32_t)(0.75f * adv_interval_units(cfg->bt5_rate_hz, 1)),
        .interval_max = adv_interval_units(cfg->bt5_rate_hz, 1),
        .channel_map = ADV_CHNL_ALL,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST,
        .tx_power = clamp_power(cfg->bt5_power_dbm),
        .primary_phy = ESP_BLE_GAP_PHY_CODED,
        .max_skip = 0,
        .secondary_phy = ESP_BLE_GAP_PHY_CODED,
        .sid = 1,
        .scan_req_notif = false,
    };

    err = esp_ble_gap_set_preferred_default_phy(ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING,
                                                ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE preferred PHY setup failed: %s", esp_err_to_name(err));
    }

    s_adv_configured[0] = configure_adv_set(0, &legacy, legacy_mac);
    s_adv_configured[1] = configure_adv_set(1, &coded, coded_mac);
    if (!s_adv_configured[0] && !s_adv_configured[1]) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void ensure_adv_started(uint8_t instance) {
    if (instance >= 2 || !s_adv_configured[instance] || s_adv_started[instance]) {
        return;
    }
    const esp_ble_gap_ext_adv_t adv = {
        .instance = instance,
        .duration = 0,
        .max_events = 0,
    };
    gap_prepare_wait(ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, instance);
    esp_err_t err = gap_call_wait(instance,
                                  "start",
                                  esp_ble_gap_ext_adv_start(1, &adv));
    if (err == ESP_OK) {
        s_adv_started[instance] = true;
    }
}

static bool transmit_long_range(ODID_UAS_Data *uas) {
    if (!s_adv_configured[1]) {
        return true;
    }
    uint8_t payload[250] = {0};
    int length = odid_message_build_pack(uas, payload, sizeof(payload));
    if (length <= 0 || length + 6 > 251) {
        return false;
    }

    uint8_t adv[251] = {0};
    adv[0] = (uint8_t)(length + 5);
    adv[1] = 0x16;
    adv[2] = 0xFA;
    adv[3] = 0xFF;
    adv[4] = 0x0D;
    adv[5] = s_msg_counters[ODID_MSG_COUNTER_PACKED]++;
    memcpy(&adv[6], payload, (size_t)length);

    gap_prepare_wait(ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT, 1);
    if (gap_call_wait(1,
                      "data",
                      esp_ble_gap_config_ext_adv_data_raw(1, (uint16_t)(length + 6), adv)) != ESP_OK) {
        return false;
    }
    ensure_adv_started(1);
    return true;
}

static bool encode_legacy_phase(ODID_UAS_Data *uas, uint8_t payload[31], uint8_t *length) {
    const uint8_t header[] = {0x1E, 0x16, 0xFA, 0xFF, 0x0D};
    memset(payload, 0, 31);
    memcpy(payload, header, sizeof(header));
    *length = sizeof(header);

    switch (s_legacy_phase) {
    case 0:
        if (uas->LocationValid) {
            ODID_Location_encoded encoded = {0};
            if (encodeLocationMessage(&encoded, &uas->Location) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_LOCATION]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    case 1:
        if (uas->BasicIDValid[0]) {
            ODID_BasicID_encoded encoded = {0};
            if (encodeBasicIDMessage(&encoded, &uas->BasicID[0]) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_BASIC_ID]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    case 2:
        if (uas->SelfIDValid) {
            ODID_SelfID_encoded encoded = {0};
            if (encodeSelfIDMessage(&encoded, &uas->SelfID) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_SELF_ID]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    case 3:
        if (uas->SystemValid) {
            ODID_System_encoded encoded = {0};
            if (encodeSystemMessage(&encoded, &uas->System) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_SYSTEM]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    case 4:
        if (uas->OperatorIDValid) {
            ODID_OperatorID_encoded encoded = {0};
            if (encodeOperatorIDMessage(&encoded, &uas->OperatorID) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_OPERATOR_ID]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    case 5:
        if (uas->BasicIDValid[1]) {
            ODID_BasicID_encoded encoded = {0};
            if (encodeBasicIDMessage(&encoded, &uas->BasicID[1]) != ODID_SUCCESS) {
                return false;
            }
            payload[sizeof(header)] = s_msg_counters[ODID_MSG_COUNTER_BASIC_ID]++;
            memcpy(&payload[sizeof(header) + 1], &encoded, sizeof(encoded));
            *length = sizeof(header) + 1 + sizeof(encoded);
        }
        break;
    default: {
        char legacy_name[28] = {0};
        const char *uas_id = uas->BasicID[0].UASID;
        size_t id_len = strlen(uas_id);
        size_t tail = id_len < 4 ? id_len : 4;
        snprintf(legacy_name, sizeof(legacy_name), "SeryRemoteID_%s", &uas_id[id_len - tail]);
        const uint8_t name_header[] = {0x02, 0x01, 0x06, (uint8_t)(strlen(legacy_name) + 1), ESP_BLE_AD_TYPE_NAME_SHORT};
        memset(payload, 0, 31);
        memcpy(payload, name_header, sizeof(name_header));
        memcpy(&payload[sizeof(name_header)], legacy_name, strlen(legacy_name));
        *length = (uint8_t)(sizeof(name_header) + strlen(legacy_name));
        break;
    }
    }

    return true;
}

static bool transmit_legacy(ODID_UAS_Data *uas) {
    if (!s_adv_configured[0]) {
        return true;
    }
    uint8_t payload[31] = {0};
    uint8_t length = 0;
    uint8_t max_phase = uas->BasicIDValid[1] ? 7 : 6;

    bool ok = encode_legacy_phase(uas, payload, &length);
    s_legacy_phase = (uint8_t)((s_legacy_phase + 1) % max_phase);
    if (!ok || length == 0) {
        return false;
    }

    gap_prepare_wait(ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT, 0);
    if (gap_call_wait(0,
                      "data",
                      esp_ble_gap_config_ext_adv_data_raw(0, length, payload)) != ESP_OK) {
        return false;
    }
    ensure_adv_started(0);
    return true;
}

static void ble_task(void *arg) {
    (void)arg;
    uint32_t last_bt4_ms = 0;
    uint32_t last_bt5_ms = 0;
    uint32_t last_skip_log_ms = 0;

    ESP_LOGI(TAG, "BLE broadcast task started");
    while (true) {
        const rid_config_t *cfg = cfg_get();
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ODID_UAS_Data uas;
        char reason[128] = {0};

        if (!rid_state_build_uas_data(&uas, reason, sizeof(reason))) {
            if (now - last_skip_log_ms > 2000) {
                last_skip_log_ms = now;
                ESP_LOGW(TAG, "skip BLE RemoteID broadcast: %s", reason[0] ? reason : "no valid data");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint8_t states = uas.BasicIDValid[1] ? 7 : 6;
        uint32_t bt4_interval = adv_interval(cfg->bt4_rate_hz, states);
        if (bt4_interval > 0 && now - last_bt4_ms >= bt4_interval) {
            last_bt4_ms = now;
            if (!transmit_legacy(&uas)) {
                ESP_LOGW(TAG, "BLE4 legacy transmit failed");
            }
        }

        uint32_t bt5_interval = adv_interval(cfg->bt5_rate_hz, 1);
        if (bt5_interval > 0 && now - last_bt5_ms >= bt5_interval) {
            last_bt5_ms = now;
            if (!transmit_long_range(&uas)) {
                ESP_LOGW(TAG, "BLE5 extended transmit failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t rid_ble_start(void) {
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "classic BT mem release failed: %s", esp_err_to_name(err));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_bluedroid_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_bluedroid_enable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (s_gap_sem == NULL) {
        s_gap_sem = xSemaphoreCreateBinaryStatic(&s_gap_sem_buf);
        if (s_gap_sem == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        return err;
    }

    err = configure_adv_sets();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE advertising disabled: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    xTaskCreateStatic(ble_task,
                      "rid_ble",
                      SERY_RID_TASK_STACK_WORDS,
                      NULL,
                      4,
                      s_ble_stack,
                      &s_ble_tcb);
    return ESP_OK;
}

#else

esp_err_t rid_ble_start(void) {
    ESP_LOGW(TAG, "BLE disabled in sdkconfig");
    return ESP_OK;
}

#endif
