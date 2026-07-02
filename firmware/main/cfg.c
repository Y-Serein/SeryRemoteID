#include "cfg.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "cfg";
static const char *NS = "storage";
static const char *LEGACY_NS = "sery_rid";

static nvs_handle_t s_handle;
static bool s_handle_open;

static rid_config_t s_cfg = {
    .lock_level = SERY_RID_LOCK_LEVEL_DEFAULT,
    .can_node = SERY_RID_CAN_NODE_DEFAULT,
    .can_terminate = SERY_RID_CAN_TERMINATE_DEFAULT,
    .uart_baud = SERY_RID_UART_BAUD_DEFAULT,
    .uart_tx_gpio = SERY_RID_UART_TX_GPIO,
    .uart_rx_gpio = SERY_RID_UART_RX_GPIO,
    .wifi_channel = SERY_RID_WIFI_CHANNEL_DEFAULT,
    .wifi_power_dbm = SERY_RID_WIFI_POWER_DBM_DEFAULT,
    .wifi_nan_rate_hz = SERY_RID_WIFI_NAN_RATE_HZ,
    .wifi_beacon_rate_hz = SERY_RID_WIFI_BEACON_RATE_HZ,
    .bt4_rate_hz = SERY_RID_BT4_RATE_HZ,
    .bt4_power_dbm = SERY_RID_BT4_POWER_DBM_DEFAULT,
    .bt5_rate_hz = SERY_RID_BT5_RATE_HZ,
    .bt5_power_dbm = SERY_RID_BT5_POWER_DBM_DEFAULT,
    .broadcast_powerup = SERY_RID_BROADCAST_POWERUP,
    .webserver_enable = SERY_RID_WEBSERVER_ENABLE_DEFAULT,
    .mavlink_sysid = SERY_RID_MAVLINK_SYSID_DEFAULT,
    .options = 0,
    .ua_type = 0,
    .uas_id_type = 0,
    .uas_id = "",
    .ua_type_2 = 0,
    .uas_id_type_2 = 0,
    .uas_id_2 = "",
    .wifi_ssid = "",
    .wifi_password = SERY_RID_WIFI_PASSWORD_DEFAULT,
    .to_factory_defaults = 0,
    .done_init = 0,
};

#define PARAM_DEF(_name, _type, _member, _def, _min, _max, _flags, _min_len) \
    { .info = &((const rid_param_info_t){ .name = _name, .type = _type, .flags = _flags, \
                                          .min_len = _min_len, .default_value = _def, \
                                          .min_value = _min, .max_value = _max }), \
      .value = &s_cfg._member }

static const rid_param_t s_params[] = {
    PARAM_DEF("LOCK_LEVEL", RID_PARAM_INT8, lock_level, 0, -1, 2, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("CAN_NODE", RID_PARAM_UINT8, can_node, 0, 0, 127, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("CAN_TERMINATE", RID_PARAM_UINT8, can_terminate, 0, 0, 1, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_TYPE", RID_PARAM_UINT8, ua_type, 0, 0, 15, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_ID_TYPE", RID_PARAM_UINT8, uas_id_type, 0, 0, 4, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_ID", RID_PARAM_CHAR20, uas_id, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_TYPE_2", RID_PARAM_UINT8, ua_type_2, 0, 0, 15, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_ID_TYPE_2", RID_PARAM_UINT8, uas_id_type_2, 0, 0, 4, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("UAS_ID_2", RID_PARAM_CHAR20, uas_id_2, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BAUDRATE", RID_PARAM_UINT32, uart_baud, SERY_RID_UART_BAUD_DEFAULT, 9600, 921600, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WIFI_NAN_RATE", RID_PARAM_FLOAT, wifi_nan_rate_hz, SERY_RID_WIFI_NAN_RATE_HZ, 0, 5, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WIFI_BCN_RATE", RID_PARAM_FLOAT, wifi_beacon_rate_hz, SERY_RID_WIFI_BEACON_RATE_HZ, 0, 5, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WIFI_POWER", RID_PARAM_FLOAT, wifi_power_dbm, SERY_RID_WIFI_POWER_DBM_DEFAULT, 2, 20, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BT4_RATE", RID_PARAM_FLOAT, bt4_rate_hz, SERY_RID_BT4_RATE_HZ, 0, 5, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BT4_POWER", RID_PARAM_FLOAT, bt4_power_dbm, SERY_RID_BT4_POWER_DBM_DEFAULT, -27, 18, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BT5_RATE", RID_PARAM_FLOAT, bt5_rate_hz, SERY_RID_BT5_RATE_HZ, 0, 5, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BT5_POWER", RID_PARAM_FLOAT, bt5_power_dbm, SERY_RID_BT5_POWER_DBM_DEFAULT, -27, 18, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WEBSERVER_EN", RID_PARAM_UINT8, webserver_enable, SERY_RID_WEBSERVER_ENABLE_DEFAULT, 0, 1, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WIFI_SSID", RID_PARAM_CHAR20, wifi_ssid, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("WIFI_PASSWORD", RID_PARAM_CHAR20, wifi_password, 0, 0, 0, RID_PARAM_FLAG_PASSWORD, 8),
    PARAM_DEF("WIFI_CHANNEL", RID_PARAM_UINT8, wifi_channel, SERY_RID_WIFI_CHANNEL_DEFAULT, 1, 13, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("BCAST_POWERUP", RID_PARAM_UINT8, broadcast_powerup, SERY_RID_BROADCAST_POWERUP ? 1 : 0, 0, 1, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("PUBLIC_KEY1", RID_PARAM_CHAR64, public_keys[0].b64_key, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("PUBLIC_KEY2", RID_PARAM_CHAR64, public_keys[1].b64_key, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("PUBLIC_KEY3", RID_PARAM_CHAR64, public_keys[2].b64_key, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("PUBLIC_KEY4", RID_PARAM_CHAR64, public_keys[3].b64_key, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("PUBLIC_KEY5", RID_PARAM_CHAR64, public_keys[4].b64_key, 0, 0, 0, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("MAVLINK_SYSID", RID_PARAM_UINT8, mavlink_sysid, SERY_RID_MAVLINK_SYSID_DEFAULT, 0, 254, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("OPTIONS", RID_PARAM_UINT8, options, 0, 0, 254, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("TO_DEFAULTS", RID_PARAM_UINT8, to_factory_defaults, 0, 0, 1, RID_PARAM_FLAG_NONE, 0),
    PARAM_DEF("DONE_INIT", RID_PARAM_UINT8, done_init, 0, 0, 0, RID_PARAM_FLAG_HIDDEN, 0),
};

static void make_default_ssid(void) {
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) {
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
    }
    snprintf(s_cfg.wifi_ssid,
             sizeof(s_cfg.wifi_ssid),
             SERY_RID_WIFI_SSID_PREFIX "_%02X%02X%02X%02X",
             mac[2],
             mac[3],
             mac[4],
             mac[5]);
}

static void write_default_value(const rid_param_t *p) {
    switch (p->info->type) {
    case RID_PARAM_UINT8:
        *(uint8_t *)p->value = (uint8_t)p->info->default_value;
        break;
    case RID_PARAM_INT8:
        *(int8_t *)p->value = (int8_t)p->info->default_value;
        break;
    case RID_PARAM_UINT32:
        *(uint32_t *)p->value = (uint32_t)p->info->default_value;
        break;
    case RID_PARAM_FLOAT:
        *(float *)p->value = p->info->default_value;
        break;
    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64:
    case RID_PARAM_NONE:
        break;
    }
}

static void load_defaults(void) {
    make_default_ssid();
    memset(s_cfg.uas_id, 0, sizeof(s_cfg.uas_id));
    memset(s_cfg.uas_id_2, 0, sizeof(s_cfg.uas_id_2));
    memset(s_cfg.public_keys, 0, sizeof(s_cfg.public_keys));
    strlcpy(s_cfg.wifi_password, SERY_RID_WIFI_PASSWORD_DEFAULT, sizeof(s_cfg.wifi_password));

    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        write_default_value(&s_params[i]);
    }
}

static bool is_float_param(const rid_param_t *p) {
    return p && !(p->info->flags & RID_PARAM_FLAG_HIDDEN) &&
           (p->info->type == RID_PARAM_UINT8 ||
            p->info->type == RID_PARAM_INT8 ||
            p->info->type == RID_PARAM_UINT32 ||
            p->info->type == RID_PARAM_FLOAT);
}

static void load_string(nvs_handle_t h, const rid_param_t *p) {
    char *dst = (char *)p->value;
    size_t len = p->info->type == RID_PARAM_CHAR64 ? 65 : 21;
    if (nvs_get_str(h, p->info->name, dst, &len) != ESP_OK) {
        return;
    }
    dst[p->info->type == RID_PARAM_CHAR64 ? 64 : 20] = 0;
}

static void load_param(nvs_handle_t h, const rid_param_t *p) {
    switch (p->info->type) {
    case RID_PARAM_UINT8: {
        uint8_t v = 0;
        if (nvs_get_u8(h, p->info->name, &v) == ESP_OK) {
            *(uint8_t *)p->value = v;
        }
        break;
    }
    case RID_PARAM_INT8: {
        int8_t v = 0;
        if (nvs_get_i8(h, p->info->name, &v) == ESP_OK) {
            *(int8_t *)p->value = v;
        }
        break;
    }
    case RID_PARAM_UINT32:
        nvs_get_u32(h, p->info->name, (uint32_t *)p->value);
        break;
    case RID_PARAM_FLOAT: {
        uint32_t raw = 0;
        if (nvs_get_u32(h, p->info->name, &raw) == ESP_OK) {
            memcpy(p->value, &raw, sizeof(float));
        }
        break;
    }
    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64:
        load_string(h, p);
        break;
    case RID_PARAM_NONE:
        break;
    }
}

static void load_legacy_short_keys(void) {
    nvs_handle_t h;
    if (nvs_open(LEGACY_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    nvs_get_u32(h, "uart_baud", &s_cfg.uart_baud);
    int32_t gpio = 0;
    if (nvs_get_i32(h, "uart_tx", &gpio) == ESP_OK) {
        s_cfg.uart_tx_gpio = gpio;
    }
    if (nvs_get_i32(h, "uart_rx", &gpio) == ESP_OK) {
        s_cfg.uart_rx_gpio = gpio;
    }
    nvs_get_u8(h, "wifi_chan", &s_cfg.wifi_channel);
    nvs_get_u8(h, "mav_sysid", &s_cfg.mavlink_sysid);
    nvs_get_u8(h, "ua_type", &s_cfg.ua_type);
    nvs_get_u8(h, "uas_id_type", &s_cfg.uas_id_type);

    uint8_t bcast = s_cfg.broadcast_powerup ? 1 : 0;
    if (nvs_get_u8(h, "bcast_pwrup", &bcast) == ESP_OK) {
        s_cfg.broadcast_powerup = bcast != 0;
    }

    uint32_t raw_float = 0;
    if (nvs_get_u32(h, "wifi_power", &raw_float) == ESP_OK) {
        memcpy(&s_cfg.wifi_power_dbm, &raw_float, sizeof(float));
    }
    if (nvs_get_u32(h, "nan_rate", &raw_float) == ESP_OK) {
        memcpy(&s_cfg.wifi_nan_rate_hz, &raw_float, sizeof(float));
    }
    if (nvs_get_u32(h, "bcn_rate", &raw_float) == ESP_OK) {
        memcpy(&s_cfg.wifi_beacon_rate_hz, &raw_float, sizeof(float));
    }

    size_t len = sizeof(s_cfg.uas_id);
    nvs_get_str(h, "uas_id", s_cfg.uas_id, &len);
    char ssid[sizeof(s_cfg.wifi_ssid)] = {0};
    len = sizeof(ssid);
    if (nvs_get_str(h, "wifi_ssid", ssid, &len) == ESP_OK && ssid[0] != 0) {
        strlcpy(s_cfg.wifi_ssid, ssid, sizeof(s_cfg.wifi_ssid));
    }
    nvs_close(h);
}

static esp_err_t save_param(const rid_param_t *p) {
    if (!s_handle_open || !p) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;
    switch (p->info->type) {
    case RID_PARAM_UINT8:
        err = nvs_set_u8(s_handle, p->info->name, *(const uint8_t *)p->value);
        break;
    case RID_PARAM_INT8:
        err = nvs_set_i8(s_handle, p->info->name, *(const int8_t *)p->value);
        break;
    case RID_PARAM_UINT32:
        err = nvs_set_u32(s_handle, p->info->name, *(const uint32_t *)p->value);
        break;
    case RID_PARAM_FLOAT: {
        uint32_t raw = 0;
        memcpy(&raw, p->value, sizeof(float));
        err = nvs_set_u32(s_handle, p->info->name, raw);
        break;
    }
    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64:
        err = nvs_set_str(s_handle, p->info->name, (const char *)p->value);
        break;
    case RID_PARAM_NONE:
        break;
    }
    if (err == ESP_OK) {
        err = nvs_commit(s_handle);
    }
    return err;
}

static bool check_range(const rid_param_t *p, float value) {
    if (!p) {
        return false;
    }
    if (p->info->type == RID_PARAM_CHAR20 ||
        p->info->type == RID_PARAM_CHAR64 ||
        p->info->type == RID_PARAM_NONE) {
        return true;
    }
    return value >= p->info->min_value && value <= p->info->max_value;
}

static void seed_default_public_keys(void) {
    cfg_param_set_by_name_char64("PUBLIC_KEY1",
                                 "PUBLIC_KEYV1:WJbbpbjOz/yMB3JxnvqyTUInCQdZcStkA0qhn2ldhPI=");
    cfg_param_set_by_name_char64("PUBLIC_KEY2",
                                 "PUBLIC_KEYV1:X8jdVqxIIUmCuMSi8IhTZ40VkXW0gbRczzMtdSghqCI=");
    cfg_param_set_by_name_char64("PUBLIC_KEY3",
                                 "PUBLIC_KEYV1:snNHkX96F9A+/ISppHZrc1jjPo3jMNN+g2PToKhWSgA=");
    cfg_param_set_by_name_char64("PUBLIC_KEY4",
                                 "PUBLIC_KEYV1:UXeWdYV39O0UlPHLo8PlRpm/qyTBT2cLX2mbj9ag4Nw=");
    cfg_param_set_by_name_char64("PUBLIC_KEY5",
                                 "PUBLIC_KEYV1:0vyB2NZwkrkLowmtXNU28Wt9JcwBPy8C9q5UvRKJAvE=");
}

void cfg_init(void) {
    load_defaults();

    esp_err_t err = nvs_open(NS, NVS_READWRITE, &s_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace '%s' open failed: %s", NS, esp_err_to_name(err));
        s_handle_open = false;
        load_legacy_short_keys();
        return;
    }
    s_handle_open = true;

    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        load_param(s_handle, &s_params[i]);
    }
    load_legacy_short_keys();

    if (s_cfg.to_factory_defaults == 1) {
        ESP_LOGW(TAG, "TO_DEFAULTS set, erasing parameter namespace and restarting");
        nvs_erase_all(s_handle);
        nvs_commit(s_handle);
        esp_restart();
    }

    if (s_cfg.done_init == 0) {
        seed_default_public_keys();
        cfg_param_set_by_name_uint8("DONE_INIT", 1);
    }

    ESP_LOGI(TAG,
             "config: uart=%lu tx=%d rx=%d wifi='%s' ch=%u nan=%.2f beacon=%.2f bt4=%.2f bt5=%.2f web=%u",
             (unsigned long)s_cfg.uart_baud,
             s_cfg.uart_tx_gpio,
             s_cfg.uart_rx_gpio,
             s_cfg.wifi_ssid,
             (unsigned)s_cfg.wifi_channel,
             (double)s_cfg.wifi_nan_rate_hz,
             (double)s_cfg.wifi_beacon_rate_hz,
             (double)s_cfg.bt4_rate_hz,
             (double)s_cfg.bt5_rate_hz,
             (unsigned)s_cfg.webserver_enable);
}

const rid_config_t *cfg_get(void) {
    return &s_cfg;
}

bool cfg_have_basic_id(void) {
    return s_cfg.ua_type > 0 && s_cfg.uas_id_type > 0 && s_cfg.uas_id[0] != 0;
}

bool cfg_have_basic_id_2(void) {
    return s_cfg.ua_type_2 > 0 && s_cfg.uas_id_type_2 > 0 && s_cfg.uas_id_2[0] != 0;
}

const rid_param_t *cfg_param_find(const char *name) {
    if (!name) {
        return NULL;
    }
    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        if (strcmp(name, s_params[i].info->name) == 0) {
            return &s_params[i];
        }
    }
    return NULL;
}

const rid_param_t *cfg_param_find_by_index(uint16_t index) {
    if (index >= sizeof(s_params) / sizeof(s_params[0])) {
        return NULL;
    }
    return &s_params[index];
}

const rid_param_t *cfg_param_find_float_by_index(uint16_t index) {
    uint16_t count = 0;
    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        if (!is_float_param(&s_params[i])) {
            continue;
        }
        if (count == index) {
            return &s_params[i];
        }
        count++;
    }
    return NULL;
}

uint16_t cfg_param_count_float(void) {
    uint16_t count = 0;
    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        if (is_float_param(&s_params[i])) {
            count++;
        }
    }
    return count;
}

int16_t cfg_param_index_float(const rid_param_t *param) {
    uint16_t count = 0;
    for (size_t i = 0; i < sizeof(s_params) / sizeof(s_params[0]); i++) {
        if (!is_float_param(&s_params[i])) {
            continue;
        }
        if (param == &s_params[i]) {
            return (int16_t)count;
        }
        count++;
    }
    return -1;
}

bool cfg_param_get_as_float(const rid_param_t *param, float *value) {
    if (!param || !value) {
        return false;
    }
    switch (param->info->type) {
    case RID_PARAM_UINT8:
        *value = (float)*(const uint8_t *)param->value;
        return true;
    case RID_PARAM_INT8:
        *value = (float)*(const int8_t *)param->value;
        return true;
    case RID_PARAM_UINT32:
        *value = (float)*(const uint32_t *)param->value;
        return true;
    case RID_PARAM_FLOAT:
        *value = *(const float *)param->value;
        return true;
    default:
        return false;
    }
}

bool cfg_param_set_as_float(const rid_param_t *param, float value) {
    if (!check_range(param, value)) {
        return false;
    }
    switch (param->info->type) {
    case RID_PARAM_UINT8:
        *(uint8_t *)param->value = (uint8_t)value;
        break;
    case RID_PARAM_INT8:
        *(int8_t *)param->value = (int8_t)value;
        break;
    case RID_PARAM_UINT32:
        *(uint32_t *)param->value = (uint32_t)value;
        break;
    case RID_PARAM_FLOAT:
        *(float *)param->value = value;
        break;
    default:
        return false;
    }
    return save_param(param) == ESP_OK;
}

static bool set_string_param(const rid_param_t *param, const char *value) {
    if (!param || !value) {
        return false;
    }
    if (param->info->type != RID_PARAM_CHAR20 && param->info->type != RID_PARAM_CHAR64) {
        return false;
    }
    if (param->info->min_len > 0 && strlen(value) < param->info->min_len) {
        return false;
    }
    size_t limit = param->info->type == RID_PARAM_CHAR64 ? 64 : 20;
    char *dst = (char *)param->value;
    memset(dst, 0, limit + 1);
    strlcpy(dst, value, limit + 1);
    return save_param(param) == ESP_OK;
}

bool cfg_param_set_by_name_string(const char *name, const char *value) {
    const rid_param_t *param = cfg_param_find(name);
    if (!param || !value) {
        return false;
    }
    switch (param->info->type) {
    case RID_PARAM_UINT8:
    case RID_PARAM_INT8:
    case RID_PARAM_UINT32:
    case RID_PARAM_FLOAT:
        return cfg_param_set_as_float(param, strtof(value, NULL));
    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64:
        return set_string_param(param, value);
    default:
        return false;
    }
}

bool cfg_param_set_by_name_uint8(const char *name, uint8_t value) {
    const rid_param_t *param = cfg_param_find(name);
    if (!param || param->info->type != RID_PARAM_UINT8) {
        return false;
    }
    return cfg_param_set_as_float(param, (float)value);
}

bool cfg_param_set_by_name_char64(const char *name, const char *value) {
    const rid_param_t *param = cfg_param_find(name);
    if (!param || param->info->type != RID_PARAM_CHAR64) {
        return false;
    }
    return set_string_param(param, value);
}

const char *cfg_param_get_string(const rid_param_t *param) {
    if (!param) {
        return NULL;
    }
    if (param->info->type != RID_PARAM_CHAR20 && param->info->type != RID_PARAM_CHAR64) {
        return NULL;
    }
    if (param->info->flags & RID_PARAM_FLAG_PASSWORD) {
        return "********";
    }
    return (const char *)param->value;
}

static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int32_t base64_decode(const char *s, uint8_t *out, uint32_t max_len) {
    const char *p;
    uint32_t n = 0;
    uint32_t i = 0;
    memset(out, 0, max_len);
    while (*s && (p = strchr(b64, *s)) != NULL) {
        uint8_t idx = (uint8_t)(p - b64);
        uint32_t byte_offset = (i * 6) / 8;
        uint32_t bit_offset = (i * 6) % 8;
        if (byte_offset >= max_len) {
            break;
        }
        out[byte_offset] &= (uint8_t)~((1u << (8 - bit_offset)) - 1u);
        if (bit_offset < 3) {
            out[byte_offset] |= (uint8_t)(idx << (2 - bit_offset));
            n = byte_offset + 1;
        } else {
            out[byte_offset] |= (uint8_t)(idx >> (bit_offset - 2));
            n = byte_offset + 1;
            if (byte_offset + 1 >= max_len) {
                break;
            }
            out[byte_offset + 1] = (uint8_t)((idx << (8 - (bit_offset - 2))) & 0xFFu);
            n = byte_offset + 2;
        }
        s++;
        i++;
    }
    if (n > 0 && *s == '=') {
        n--;
    }
    return (int32_t)n;
}

static bool base64_encode32(const uint8_t *d, char out[45]) {
    uint32_t i = 0;
    uint32_t bytes = (32 * 8 + 5) / 6;
    uint32_t pad_bytes = (bytes % 4) ? 4 - (bytes % 4) : 0;
    for (; i < bytes; i++) {
        uint32_t byte_offset = (i * 6) / 8;
        uint32_t bit_offset = (i * 6) % 8;
        uint32_t idx;
        if (bit_offset < 3) {
            idx = (d[byte_offset] >> (2 - bit_offset)) & 0x3Fu;
        } else {
            idx = (d[byte_offset] << (bit_offset - 2)) & 0x3Fu;
            if (byte_offset + 1 < 32) {
                idx |= d[byte_offset + 1] >> (8 - (bit_offset - 2));
            }
        }
        out[i] = b64[idx];
    }
    for (; i < bytes + pad_bytes; i++) {
        out[i] = '=';
    }
    out[i] = 0;
    return true;
}

bool cfg_get_public_key(uint8_t index, uint8_t key[32]) {
    if (index >= 5 || !key) {
        return false;
    }
    const char *stored = s_cfg.public_keys[index].b64_key;
    const char *prefix = "PUBLIC_KEYV1:";
    if (strncmp(stored, prefix, strlen(prefix)) == 0) {
        stored += strlen(prefix);
    }
    int32_t len = base64_decode(stored, key, 32);
    return len == 32;
}

bool cfg_set_public_key(uint8_t index, const uint8_t key[32]) {
    if (index >= 5 || !key) {
        return false;
    }
    char encoded[45] = {0};
    base64_encode32(key, encoded);
    char stored[65] = {0};
    snprintf(stored, sizeof(stored), "PUBLIC_KEYV1:%s", encoded);
    char name[] = "PUBLIC_KEY1";
    name[10] = (char)('1' + index);
    return cfg_param_set_by_name_char64(name, stored);
}

bool cfg_remove_public_key(uint8_t index) {
    if (index >= 5) {
        return false;
    }
    char name[] = "PUBLIC_KEY1";
    name[10] = (char)('1' + index);
    return cfg_param_set_by_name_char64(name, "");
}

bool cfg_no_public_keys(void) {
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t key[32];
        if (cfg_get_public_key(i, key)) {
            return false;
        }
    }
    return true;
}
