#include "web/rid_web.h"

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "opendroneid.h"
#include "protocol/rid_state.h"
#include "transport/rid_mavlink.h"
#include "web/rid_firmware_check.h"

static const char *TAG = "rid_web";
static httpd_handle_t s_server;
static bool s_reboot_required;

#define CONFIG_BODY_MAX 1024
#define OTA_CHUNK_BYTES 1536

typedef enum {
    RID_OTA_DISABLED = 0,
    RID_OTA_READY,
    RID_OTA_UPLOADING,
    RID_OTA_VERIFYING,
    RID_OTA_SUCCESS,
    RID_OTA_ERROR,
} rid_ota_state_t;

static volatile bool s_ota_enabled;
static volatile bool s_ota_active;
static volatile rid_ota_state_t s_ota_state = RID_OTA_DISABLED;
static volatile uint8_t s_ota_progress;
static char s_ota_error[96];
static char s_ota_chunk[OTA_CHUNK_BYTES];

typedef struct {
    const esp_partition_t *part;
    esp_ota_handle_t handle;
    bool started;
    esp_err_t err;
    size_t written;
} ota_upload_t;

typedef struct {
    int value;
    const char *name;
} enum_name_t;

typedef struct {
    uint8_t input_profile;
    uint32_t baudrate;
    uint8_t mavlink_sysid;
    uint8_t uas_type;
    uint8_t uas_id_type;
    char uas_id[21];
} web_config_t;

#define ARRAY_LEN(_a) (sizeof(_a) / sizeof((_a)[0]))

static const enum_name_t ENUM_UATYPE[] = {
    { ODID_UATYPE_NONE, "NONE" },
    { ODID_UATYPE_AEROPLANE, "AEROPLANE" },
    { ODID_UATYPE_HELICOPTER_OR_MULTIROTOR, "HELICOPTER_OR_MULTIROTOR" },
    { ODID_UATYPE_GYROPLANE, "GYROPLANE" },
    { ODID_UATYPE_HYBRID_LIFT, "HYBRID_LIFT" },
    { ODID_UATYPE_ORNITHOPTER, "ORNITHOPTER" },
    { ODID_UATYPE_GLIDER, "GLIDER" },
    { ODID_UATYPE_KITE, "KITE" },
    { ODID_UATYPE_FREE_BALLOON, "FREE_BALLOON" },
    { ODID_UATYPE_CAPTIVE_BALLOON, "CAPTIVE_BALLOON" },
    { ODID_UATYPE_AIRSHIP, "AIRSHIP" },
    { ODID_UATYPE_FREE_FALL_PARACHUTE, "FREE_FALL_PARACHUTE" },
    { ODID_UATYPE_ROCKET, "ROCKET" },
    { ODID_UATYPE_TETHERED_POWERED_AIRCRAFT, "TETHERED_POWERED_AIRCRAFT" },
    { ODID_UATYPE_GROUND_OBSTACLE, "GROUND_OBSTACLE" },
    { ODID_UATYPE_OTHER, "OTHER" },
};

static const enum_name_t ENUM_IDTYPE[] = {
    { ODID_IDTYPE_NONE, "NONE" },
    { ODID_IDTYPE_SERIAL_NUMBER, "SERIAL_NUMBER" },
    { ODID_IDTYPE_CAA_REGISTRATION_ID, "CAA_REGISTRATION_ID" },
    { ODID_IDTYPE_UTM_ASSIGNED_UUID, "UTM_ASSIGNED_UUID" },
    { ODID_IDTYPE_SPECIFIC_SESSION_ID, "SPECIFIC_SESSION_ID" },
};

static const enum_name_t ENUM_LOCTYPE[] = {
    { ODID_OPERATOR_LOCATION_TYPE_TAKEOFF, "TAKEOFF" },
    { ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS, "LIVE_GNSS" },
    { ODID_OPERATOR_LOCATION_TYPE_FIXED, "FIXED" },
};

static const enum_name_t ENUM_CLASSIF[] = {
    { ODID_CLASSIFICATION_TYPE_UNDECLARED, "UNDECLARED" },
    { ODID_CLASSIFICATION_TYPE_EU, "EU" },
};

static const enum_name_t ENUM_STATUS[] = {
    { ODID_STATUS_UNDECLARED, "UNDECLARED" },
    { ODID_STATUS_GROUND, "GROUND" },
    { ODID_STATUS_AIRBORNE, "AIRBORNE" },
    { ODID_STATUS_EMERGENCY, "EMERGENCY" },
    { ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE, "REMOTE_ID_SYSTEM_FAILURE" },
};

static const enum_name_t ENUM_HEIGHT[] = {
    { ODID_HEIGHT_REF_OVER_TAKEOFF, "OVER_TAKEOFF" },
    { ODID_HEIGHT_REF_OVER_GROUND, "OVER_GROUND" },
};

static const enum_name_t ENUM_HACC[] = {
    { ODID_HOR_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_HOR_ACC_10NM, "10 nm" },
    { ODID_HOR_ACC_4NM, "4 nm" },
    { ODID_HOR_ACC_2NM, "2 nm" },
    { ODID_HOR_ACC_1NM, "1 nm" },
    { ODID_HOR_ACC_0_5NM, "0.5 nm" },
    { ODID_HOR_ACC_0_3NM, "0.3 nm" },
    { ODID_HOR_ACC_0_1NM, "0.1 nm" },
    { ODID_HOR_ACC_0_05NM, "0.05 nm" },
    { ODID_HOR_ACC_30_METER, "30 m" },
    { ODID_HOR_ACC_10_METER, "10 m" },
    { ODID_HOR_ACC_3_METER, "3 m" },
    { ODID_HOR_ACC_1_METER, "1 m" },
};

static const enum_name_t ENUM_VACC[] = {
    { ODID_VER_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_VER_ACC_150_METER, "150 m" },
    { ODID_VER_ACC_45_METER, "45 m" },
    { ODID_VER_ACC_25_METER, "25 m" },
    { ODID_VER_ACC_10_METER, "10 m" },
    { ODID_VER_ACC_3_METER, "3 m" },
    { ODID_VER_ACC_1_METER, "1 m" },
};

static const enum_name_t ENUM_SACC[] = {
    { ODID_SPEED_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_SPEED_ACC_10_METERS_PER_SECOND, "10 m/s" },
    { ODID_SPEED_ACC_3_METERS_PER_SECOND, "3 m/s" },
    { ODID_SPEED_ACC_1_METERS_PER_SECOND, "1 m/s" },
    { ODID_SPEED_ACC_0_3_METERS_PER_SECOND, "0.3 m/s" },
};

static const enum_name_t ENUM_DESCTYPE[] = {
    { ODID_DESC_TYPE_TEXT, "TEXT" },
    { ODID_DESC_TYPE_EMERGENCY, "EMERGENCY" },
    { ODID_DESC_TYPE_EXTENDED_STATUS, "EXTENDED_STATUS" },
};

static const enum_name_t ENUM_CLASSEU[] = {
    { ODID_CLASS_EU_UNDECLARED, "UNDECLARED" },
    { ODID_CLASS_EU_CLASS_0, "CLASS_0" },
    { ODID_CLASS_EU_CLASS_1, "CLASS_1" },
    { ODID_CLASS_EU_CLASS_2, "CLASS_2" },
    { ODID_CLASS_EU_CLASS_3, "CLASS_3" },
    { ODID_CLASS_EU_CLASS_4, "CLASS_4" },
    { ODID_CLASS_EU_CLASS_5, "CLASS_5" },
    { ODID_CLASS_EU_CLASS_6, "CLASS_6" },
};

static const enum_name_t ENUM_CATEU[] = {
    { ODID_CATEGORY_EU_UNDECLARED, "UNDECLARED" },
    { ODID_CATEGORY_EU_OPEN, "OPEN" },
    { ODID_CATEGORY_EU_SPECIFIC, "SPECIFIC" },
    { ODID_CATEGORY_EU_CERTIFIED, "CERTIFIED" },
};

static const enum_name_t ENUM_TSACC[] = {
    { ODID_TIME_ACC_UNKNOWN, "UNKNOWN" },
    { ODID_TIME_ACC_0_1_SECOND, "0.1 s" },
    { ODID_TIME_ACC_0_2_SECOND, "0.2 s" },
    { ODID_TIME_ACC_0_3_SECOND, "0.3 s" },
    { ODID_TIME_ACC_0_4_SECOND, "0.4 s" },
    { ODID_TIME_ACC_0_5_SECOND, "0.5 s" },
    { ODID_TIME_ACC_0_6_SECOND, "0.6 s" },
    { ODID_TIME_ACC_0_7_SECOND, "0.7 s" },
    { ODID_TIME_ACC_0_8_SECOND, "0.8 s" },
    { ODID_TIME_ACC_0_9_SECOND, "0.9 s" },
    { ODID_TIME_ACC_1_0_SECOND, "1.0 s" },
    { ODID_TIME_ACC_1_1_SECOND, "1.1 s" },
    { ODID_TIME_ACC_1_2_SECOND, "1.2 s" },
    { ODID_TIME_ACC_1_3_SECOND, "1.3 s" },
    { ODID_TIME_ACC_1_4_SECOND, "1.4 s" },
    { ODID_TIME_ACC_1_5_SECOND, "1.5 s" },
};

static bool appendf(char **cursor, size_t *remaining, const char *fmt, ...) {
    if (!cursor || !*cursor || !remaining || *remaining == 0) {
        return false;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(*cursor, *remaining, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return false;
    }
    if ((size_t)n >= *remaining) {
        *cursor += *remaining - 1;
        *remaining = 1;
        return false;
    }
    *cursor += n;
    *remaining -= (size_t)n;
    return true;
}

static void json_value_escaped(char **cursor, size_t *remaining, const char *value) {
    const unsigned char *s = (const unsigned char *)(value ? value : "");
    while (*s != 0 && remaining && *remaining > 1) {
        if (*s == '"' || *s == '\\') {
            appendf(cursor, remaining, "\\%c", *s);
        } else if (*s >= 0x20) {
            appendf(cursor, remaining, "%c", *s);
        }
        s++;
    }
}

static void json_add_string(char **cursor,
                            size_t *remaining,
                            bool *first,
                            const char *key,
                            const char *value) {
    appendf(cursor, remaining, "%s\"%s\":\"", *first ? "" : ",", key);
    json_value_escaped(cursor, remaining, value);
    appendf(cursor, remaining, "\"");
    *first = false;
}

static void json_add_fmt(char **cursor,
                         size_t *remaining,
                         bool *first,
                         const char *key,
                         const char *fmt,
                         ...) {
    char value[64] = {0};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(value, sizeof(value), fmt, ap);
    va_end(ap);
    json_add_string(cursor, remaining, first, key, value);
}

static const char *enum_name(const enum_name_t *map,
                             size_t map_len,
                             int value,
                             char fallback[16]) {
    for (size_t i = 0; i < map_len; i++) {
        if (map[i].value == value) {
            return map[i].name;
        }
    }
    snprintf(fallback, 16, "%d", value);
    return fallback;
}

static void json_add_enum(char **cursor,
                          size_t *remaining,
                          bool *first,
                          const char *key,
                          const enum_name_t *map,
                          size_t map_len,
                          int value) {
    char fallback[16] = {0};
    json_add_string(cursor, remaining, first, key, enum_name(map, map_len, value, fallback));
}

static const char *input_profile_name(uint8_t profile) {
    switch (profile) {
    case RID_INPUT_PROFILE_GENERIC_MAVLINK:
        return "Generic MAVLink2";
    case RID_INPUT_PROFILE_LINGDONG_A7:
        return "Lingdong A7";
    case RID_INPUT_PROFILE_CUSTOM_MAVLINK:
        return "Custom MAVLink2";
    default:
        return "Unknown";
    }
}

static bool config_authorized(httpd_req_t *req) {
    char supplied[32] = {0};
    if (httpd_req_get_hdr_value_str(req,
                                    "X-Sery-Config-Key",
                                    supplied,
                                    sizeof(supplied)) != ESP_OK) {
        return false;
    }
    return supplied[0] != 0 && strcmp(supplied, cfg_get()->wifi_password) == 0;
}

static esp_err_t require_config_authorization(httpd_req_t *req) {
    if (config_authorized(req)) {
        return ESP_OK;
    }
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "invalid configuration key");
    return ESP_FAIL;
}

static esp_err_t receive_request_body(httpd_req_t *req, char *body, size_t body_size) {
    if (!body || body_size < 2 || req->content_len <= 0 ||
        req->content_len >= (int)body_size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid request body");
        return ESP_FAIL;
    }

    size_t offset = 0;
    while (offset < (size_t)req->content_len) {
        int received = httpd_req_recv(req,
                                      body + offset,
                                      (size_t)req->content_len - offset);
        if (received <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "incomplete request body");
            return ESP_FAIL;
        }
        offset += (size_t)received;
    }
    body[offset] = 0;
    return ESP_OK;
}

static bool parse_u32(const char *value, uint32_t min, uint32_t max, uint32_t *out) {
    if (!value || !*value || !out) {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (errno != 0 || !end || *end != 0 || parsed < min || parsed > max) {
        return false;
    }
    *out = (uint32_t)parsed;
    return true;
}

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static bool url_decode(char *value) {
    char *src = value;
    char *dst = value;
    while (src && *src) {
        if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else if (*src == '%') {
            int high = hex_digit(src[1]);
            int low = hex_digit(src[2]);
            if (high < 0 || low < 0) {
                return false;
            }
            *dst++ = (char)((high << 4) | low);
            src += 3;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = 0;
    return true;
}

static bool form_value(const char *body, const char *key, char *out, size_t out_size) {
    if (httpd_query_key_value(body, key, out, out_size) != ESP_OK) {
        return false;
    }
    return url_decode(out);
}

static void web_config_load(web_config_t *out) {
    const rid_config_t *cfg = cfg_get();
    memset(out, 0, sizeof(*out));
    out->input_profile = cfg->input_profile;
    out->baudrate = cfg->uart_baud;
    out->mavlink_sysid = cfg->mavlink_sysid;
    out->uas_type = cfg->ua_type;
    out->uas_id_type = cfg->uas_id_type;
    strlcpy(out->uas_id, cfg->uas_id, sizeof(out->uas_id));
}

static bool valid_uas_id(const char *value) {
    if (!value) {
        return false;
    }
    for (const unsigned char *p = (const unsigned char *)value; *p; p++) {
        if (!isalnum(*p) && *p != '-' && *p != '_' && *p != '.' && *p != ':') {
            return false;
        }
    }
    return true;
}

static bool web_config_validate(web_config_t *config) {
    if (!config || config->input_profile > RID_INPUT_PROFILE_CUSTOM_MAVLINK ||
        config->baudrate < 9600 || config->baudrate > 921600 ||
        config->mavlink_sysid > 254 || config->uas_type > 15 ||
        config->uas_id_type > 4 || strlen(config->uas_id) > 20 ||
        !valid_uas_id(config->uas_id)) {
        return false;
    }
    if (config->input_profile == RID_INPUT_PROFILE_LINGDONG_A7) {
        config->baudrate = 57600;
    }
    if (config->uas_id_type != 0 && config->uas_id[0] == 0) {
        return false;
    }
    return true;
}

static bool web_config_apply(web_config_t *config) {
    if (!web_config_validate(config)) {
        return false;
    }

    const rid_config_t *current = cfg_get();
    bool input_changed = current->input_profile != config->input_profile ||
                         current->uart_baud != config->baudrate ||
                         current->mavlink_sysid != config->mavlink_sysid;
    char baudrate[16] = {0};
    snprintf(baudrate, sizeof(baudrate), "%lu", (unsigned long)config->baudrate);

    if (!cfg_param_set_by_name_uint8("INPUT_PROFILE", config->input_profile) ||
        !cfg_param_set_by_name_string("BAUDRATE", baudrate) ||
        !cfg_param_set_by_name_uint8("MAVLINK_SYSID", config->mavlink_sysid) ||
        !cfg_param_set_by_name_uint8("UAS_TYPE", config->uas_type) ||
        !cfg_param_set_by_name_uint8("UAS_ID_TYPE", config->uas_id_type) ||
        !cfg_param_set_by_name_string("UAS_ID", config->uas_id)) {
        return false;
    }

    s_reboot_required = s_reboot_required || input_changed;
    return true;
}

static bool parse_form_config(const char *body, web_config_t *config) {
    char value[64] = {0};
    uint32_t parsed = 0;
    web_config_load(config);

    if (!form_value(body, "input_profile", value, sizeof(value)) ||
        !parse_u32(value, 0, RID_INPUT_PROFILE_CUSTOM_MAVLINK, &parsed)) {
        return false;
    }
    config->input_profile = (uint8_t)parsed;

    if (!form_value(body, "baudrate", value, sizeof(value)) ||
        !parse_u32(value, 9600, 921600, &config->baudrate)) {
        return false;
    }
    if (!form_value(body, "mavlink_sysid", value, sizeof(value)) ||
        !parse_u32(value, 0, 254, &parsed)) {
        return false;
    }
    config->mavlink_sysid = (uint8_t)parsed;
    if (!form_value(body, "uas_type", value, sizeof(value)) ||
        !parse_u32(value, 0, 15, &parsed)) {
        return false;
    }
    config->uas_type = (uint8_t)parsed;
    if (!form_value(body, "uas_id_type", value, sizeof(value)) ||
        !parse_u32(value, 0, 4, &parsed)) {
        return false;
    }
    config->uas_id_type = (uint8_t)parsed;
    if (!form_value(body, "uas_id", config->uas_id, sizeof(config->uas_id))) {
        return false;
    }
    return true;
}

static char *trim_space(char *value) {
    while (value && isspace((unsigned char)*value)) {
        value++;
    }
    if (!value) {
        return NULL;
    }
    char *end = value + strlen(value);
    while (end > value && isspace((unsigned char)end[-1])) {
        *--end = 0;
    }
    return value;
}

static bool parse_import_config(char *body, web_config_t *config) {
    web_config_load(config);
    bool found_value = false;
    char *saveptr = NULL;
    for (char *line = strtok_r(body, "\r\n", &saveptr);
         line;
         line = strtok_r(NULL, "\r\n", &saveptr)) {
        line = trim_space(line);
        if (!*line || *line == '#' || strcmp(line, "SERYRID_CONFIG_V1") == 0) {
            continue;
        }
        char *separator = strchr(line, '=');
        if (!separator) {
            return false;
        }
        *separator = 0;
        char *key = trim_space(line);
        char *value = trim_space(separator + 1);
        uint32_t parsed = 0;

        if (strcmp(key, "INPUT_PROFILE") == 0 &&
            parse_u32(value, 0, RID_INPUT_PROFILE_CUSTOM_MAVLINK, &parsed)) {
            config->input_profile = (uint8_t)parsed;
        } else if (strcmp(key, "BAUDRATE") == 0 &&
                   parse_u32(value, 9600, 921600, &config->baudrate)) {
        } else if (strcmp(key, "MAVLINK_SYSID") == 0 &&
                   parse_u32(value, 0, 254, &parsed)) {
            config->mavlink_sysid = (uint8_t)parsed;
        } else if (strcmp(key, "UAS_TYPE") == 0 &&
                   parse_u32(value, 0, 15, &parsed)) {
            config->uas_type = (uint8_t)parsed;
        } else if (strcmp(key, "UAS_ID_TYPE") == 0 &&
                   parse_u32(value, 0, 4, &parsed)) {
            config->uas_id_type = (uint8_t)parsed;
        } else if (strcmp(key, "UAS_ID") == 0 && strlen(value) <= 20) {
            strlcpy(config->uas_id, value, sizeof(config->uas_id));
        } else {
            return false;
        }
        found_value = true;
    }
    return found_value;
}

static const char *latlon_string(double lat, double lon, bool latitude, char out[24]) {
    if (lat == 0.0 && lon == 0.0) {
        return "UNKNOWN";
    }
    snprintf(out, 24, "%.8f", latitude ? lat : lon);
    return out;
}

static const char *alt_string(float alt, char out[24]) {
    if (alt <= -1000.0f) {
        return "UNKNOWN";
    }
    snprintf(out, 24, "%.2f", (double)alt);
    return out;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    static const char html[] =
        "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>SeryRemoteID</title><style>"
        ":root{--bg:#f5f7fa;--panel:#fff;--ink:#18202a;--muted:#687386;--line:#d9e0e8;--ok:#16784a;--bad:#b42318;--warn:#9a6700;--chip:#edf1f5}"
        "*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:\"Segoe UI\",Tahoma,sans-serif;letter-spacing:0}"
        ".wrap{max-width:1120px;margin:0 auto;padding:20px}.top{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;margin-bottom:16px}"
        "h1{margin:0;font-size:28px;line-height:1.1}h2{margin:0 0 12px;font-size:15px}.sub{margin-top:6px;color:var(--muted);font-size:13px}"
        ".badge{border:1px solid var(--line);border-radius:6px;background:var(--panel);padding:10px 12px;min-width:118px;text-align:center;font-weight:700}.badge.ok{color:var(--ok);border-color:#8fd5b2}.badge.bad{color:var(--bad);border-color:#efb1aa}"
        ".summary{display:grid;grid-template-columns:1.2fr 2fr;gap:12px;margin-bottom:12px}.hero,.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:14px}"
        ".hero strong{display:block;font-size:20px;margin-top:4px}.chips{display:flex;flex-wrap:wrap;gap:6px}.chip{background:var(--chip);border:1px solid var(--line);border-radius:999px;padding:4px 8px;font-size:12px;color:var(--muted)}.chip.bad{color:var(--bad);background:#fff1f0;border-color:#f3bbb5}"
        ".grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.wide{grid-column:span 2}.kv{display:grid;grid-template-columns:minmax(96px,44%) minmax(0,1fr);gap:10px;padding:7px 0;border-top:1px solid #eef2f6}.kv:first-child{border-top:0}.kv span{color:var(--muted);font-size:12px}.kv b{font-size:13px;overflow-wrap:anywhere;font-weight:650}"
        ".ota{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.btn{display:inline-block;border:1px solid #1f2937;background:#1f2937;color:#fff;border-radius:6px;padding:8px 12px;font-weight:700;cursor:pointer;text-decoration:none}.btn:disabled{opacity:.55;cursor:not-allowed}"
        ".formgrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.field{display:flex;flex-direction:column;gap:5px}.field label{font-size:12px;color:var(--muted)}.field input,.field select{width:100%;border:1px solid var(--line);border-radius:6px;background:#fff;color:var(--ink);padding:8px;font-size:13px}.actions{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:12px}.btn.alt{background:#fff;color:#1f2937}.hint{margin-top:10px;padding:9px 10px;border-left:3px solid #d2a106;background:#fff8db;color:#6f5200;font-size:12px;line-height:1.5}.statusline{font-size:12px;color:var(--muted)}"
        ".raw{white-space:pre-wrap;background:#111827;color:#d1fae5;border-radius:6px;padding:12px;overflow:auto;max-height:340px;font:12px/1.45 ui-monospace,Consolas,monospace}"
        "details{margin-top:12px}summary{cursor:pointer;color:var(--muted);font-size:13px}@media(max-width:760px){.wrap{padding:14px}.top,.summary{display:block}.badge{margin-top:12px}.grid,.formgrid{grid-template-columns:1fr}.wide{grid-column:auto}}"
        "</style></head><body><main class=\"wrap\">"
        "<header class=\"top\"><div><h1>SeryRemoteID</h1><div class=\"sub\"><span id=\"app\">sery_remoteid</span> | <span id=\"version\">-</span> | <span id=\"uptime\">--:--:--</span></div></div><div id=\"badge\" class=\"badge bad\">WAITING</div></header>"
        "<section class=\"summary\"><div class=\"hero\"><span>RemoteID arm state</span><strong id=\"armText\">Waiting for data</strong></div><div class=\"panel\"><h2>Missing Inputs</h2><div id=\"chips\" class=\"chips\"></div></div></section>"
        "<section class=\"grid\">"
        "<div class=\"panel\"><h2>Device</h2><div id=\"deviceRows\"></div></div>"
        "<div class=\"panel\"><h2>Basic ID</h2><div id=\"basicRows\"></div></div>"
        "<div class=\"panel\"><h2>Location</h2><div id=\"locationRows\"></div></div>"
        "<div class=\"panel\"><h2>Operator</h2><div id=\"operatorRows\"></div></div>"
        "<div class=\"panel\"><h2>System</h2><div id=\"systemRows\"></div></div>"
        "<div class=\"panel\"><h2>Firmware Update</h2><div class=\"ota\"><a class=\"btn\" href=\"/ota\">打开 OTA 页面</a><span class=\"statusline\">升级默认关闭，需要配置口令显式开启。</span></div></div>"
        "<div class=\"panel wide\"><h2>Input Configuration</h2><div class=\"formgrid\">"
        "<div class=\"field\"><label for=\"inputProfile\">飞控预设</label><select id=\"inputProfile\"><option value=\"0\">通用 MAVLink2</option><option value=\"1\">凌动 A7</option><option value=\"2\">自定义 MAVLink2</option></select></div>"
        "<div class=\"field\"><label for=\"baudrate\">串口波特率</label><input id=\"baudrate\" type=\"number\" min=\"9600\" max=\"921600\" step=\"1\"></div>"
        "<div class=\"field\"><label for=\"mavSysid\">MAVLink SYSID（0=自动学习）</label><input id=\"mavSysid\" type=\"number\" min=\"0\" max=\"254\"></div>"
        "<div class=\"field\"><label for=\"uasType\">UAS Type（0-15）</label><input id=\"uasType\" type=\"number\" min=\"0\" max=\"15\"></div>"
        "<div class=\"field\"><label for=\"uasIdType\">UAS ID Type（0-4）</label><input id=\"uasIdType\" type=\"number\" min=\"0\" max=\"4\"></div>"
        "<div class=\"field\"><label for=\"uasId\">本机 UAS ID（最多20字符）</label><input id=\"uasId\" maxlength=\"20\" pattern=\"[A-Za-z0-9._:-]*\"></div>"
        "</div><div class=\"actions\"><div class=\"field\"><label for=\"configKey\">配置口令</label><input id=\"configKey\" type=\"password\" maxlength=\"20\" autocomplete=\"current-password\"></div><button id=\"saveConfig\" class=\"btn\" type=\"button\">保存配置</button><button id=\"reboot\" class=\"btn alt\" type=\"button\">重启应用</button><a class=\"btn alt\" href=\"/api/config/export\">导出配置</a><input id=\"importFile\" type=\"file\" accept=\".txt,text/plain\"><button id=\"importConfig\" class=\"btn alt\" type=\"button\">导入配置</button><span id=\"configStatus\" class=\"statusline\"></span></div>"
        "<div class=\"hint\">凌动 A7 预设固定为 MAVLink2 / 57600。A7 端仍需配置 SERIALx_PROTOCOL=2、SERIALx_BAUD=57、DID_ENABLE=1、DID_MAVPORT=x。导入文件只接受白名单配置，不执行脚本；新增私有线协议仍需 OTA 固件。</div></div>"
        "<div class=\"panel\"><h2>Input Diagnostics</h2><div id=\"diagnosticRows\"></div></div>"
        "</section><details><summary>Raw JSON</summary><pre id=\"raw\" class=\"raw\">Loading...</pre></details>"
        "</main><script>"
        "const $=id=>document.getElementById(id);"
        "const clean=v=>(v===undefined||v===null||v==='')?'-':String(v);"
        "function rows(id,defs,d){const el=$(id);el.replaceChildren();for(const [label,key] of defs){const r=document.createElement('div');r.className='kv';const s=document.createElement('span');s.textContent=label;const b=document.createElement('b');b.textContent=clean(d[key]);r.append(s,b);el.append(r);}}"
        "function chips(reason,ok){const el=$('chips');el.replaceChildren();const parts=String(reason||'').trim().split(/\\s+/).filter(Boolean);if(parts.length===0){parts.push(ok?'Complete':'Waiting');}for(const p of parts){const c=document.createElement('span');c.className='chip'+(ok?'':' bad');c.textContent=p;el.append(c);}}"
        "function render(d){const ok=d.arm_ok===true||d.arm_ok==='true';$('badge').className='badge '+(ok?'ok':'bad');$('badge').textContent=ok?'READY':'BLOCKED';$('armText').textContent=ok?'Good to arm':'Not ready';$('app').textContent=clean(d.app);$('version').textContent='v'+clean(d.version);$('uptime').textContent='uptime '+clean(d['STATUS:UPTIME']);chips(d.reason,ok);rows('deviceRows',[['WiFi SSID','wifi_ssid'],['WiFi channel','wifi_channel'],['MAVLink sysid','mavlink_sysid'],['Board ID','STATUS:BOARD_ID'],['Free heap','STATUS:FREEMEM']],d);rows('basicRows',[['UA type','BASICID:UAType'],['ID type','BASICID:IDType'],['UAS ID','BASICID:UASID'],['UA type 2','BASICID:UAType2'],['ID type 2','BASICID:IDType2'],['UAS ID 2','BASICID:UASID2']],d);rows('locationRows',[['Status','LOCATION:Status'],['Reason','LOCATION:StatusReason'],['Latitude','LOCATION:Latitude'],['Longitude','LOCATION:Longitude'],['Alt geo','LOCATION:AltitudeGeo'],['Alt baro','LOCATION:AltitudeBaro'],['Height','LOCATION:Height'],['Speed H','LOCATION:SpeedHorizontal'],['Speed V','LOCATION:SpeedVertical'],['Direction','LOCATION:Direction'],['Time','LOCATION:TimeStamp']],d);rows('operatorRows',[['Operator ID type','OPERATORID:IDType'],['Operator ID','OPERATORID:ID'],['Self ID type','SELFID:DescType'],['Self ID','SELFID:Desc']],d);rows('systemRows',[['Operator loc type','SYSTEM:OperatorLocationType'],['Operator lat','SYSTEM:OperatorLatitude'],['Operator lon','SYSTEM:OperatorLongitude'],['Area count','SYSTEM:AreaCount'],['Area radius','SYSTEM:AreaRadius'],['Ceiling','SYSTEM:AreaCeiling'],['Floor','SYSTEM:AreaFloor'],['EU category','SYSTEM:CategoryEU'],['EU class','SYSTEM:ClassEU']],d);$('raw').textContent=JSON.stringify(d,null,2);}"
        "async function load(){try{const r=await fetch('/ajax/status.json',{cache:'no-store'});render(await r.json());}catch(e){$('badge').className='badge bad';$('badge').textContent='OFFLINE';$('raw').textContent=String(e);}}"
        "function configHeaders(){return {'X-Sery-Config-Key':$('configKey').value};}"
        "function syncProfile(){const a7=$('inputProfile').value==='1';if(a7){$('baudrate').value='57600';}$('baudrate').disabled=a7;}"
        "async function loadConfig(){try{const r=await fetch('/api/config',{cache:'no-store'});const d=await r.json();$('inputProfile').value=String(d.input_profile);$('baudrate').value=String(d.baudrate);$('mavSysid').value=String(d.mavlink_sysid);$('uasType').value=String(d.uas_type);$('uasIdType').value=String(d.uas_id_type);$('uasId').value=d.uas_id||'';syncProfile();$('configStatus').textContent=d.reboot_required?'已保存，等待重启生效':'';}catch(e){$('configStatus').textContent='读取配置失败';}}"
        "async function saveConfig(){const st=$('configStatus');if(!$('configKey').value){st.textContent='请输入配置口令';return;}const p=new URLSearchParams({input_profile:$('inputProfile').value,baudrate:$('baudrate').value,mavlink_sysid:$('mavSysid').value,uas_type:$('uasType').value,uas_id_type:$('uasIdType').value,uas_id:$('uasId').value});st.textContent='保存中...';try{const r=await fetch('/api/config',{method:'POST',headers:{...configHeaders(),'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});const t=await r.text();if(!r.ok){throw new Error(t||'保存失败');}const d=JSON.parse(t);st.textContent=d.reboot_required?'保存成功，请点击重启应用':'保存成功';await loadConfig();}catch(e){st.textContent=String(e.message||e);}}"
        "async function importConfig(){const st=$('configStatus');const f=$('importFile').files[0];if(!$('configKey').value){st.textContent='请输入配置口令';return;}if(!f){st.textContent='请选择配置文件';return;}st.textContent='导入中...';try{const r=await fetch('/api/config/import',{method:'POST',headers:{...configHeaders(),'Content-Type':'text/plain'},body:await f.text()});const t=await r.text();if(!r.ok){throw new Error(t||'导入失败');}st.textContent='导入成功，请检查并重启';await loadConfig();}catch(e){st.textContent=String(e.message||e);}}"
        "async function rebootDevice(){const st=$('configStatus');if(!$('configKey').value){st.textContent='请输入配置口令';return;}if(!confirm('确认重启设备以应用串口配置？')){return;}st.textContent='正在重启...';try{await fetch('/api/reboot',{method:'POST',headers:configHeaders()});}catch(e){}setTimeout(()=>location.reload(),5000);}"
        "async function loadDiagnostics(){try{const r=await fetch('/api/input/diagnostics',{cache:'no-store'});const d=await r.json();const age=d.last_frame_age_ms===4294967295?'Never':d.last_frame_age_ms+' ms';rows('diagnosticRows',[['RX bytes','rx_bytes'],['Valid MAVLink frames','valid_frames'],['Parse errors','parse_errors'],['Last frame age','age'],['Last message ID','last_message_id'],['Source SYSID','last_system_id'],['Source COMPID','last_component_id']],{...d,age});}catch(e){rows('diagnosticRows',[['Status','status']],{status:'Unavailable'});}}"
        "$('inputProfile').addEventListener('change',syncProfile);$('saveConfig').addEventListener('click',saveConfig);$('importConfig').addEventListener('click',importConfig);$('reboot').addEventListener('click',rebootDevice);load();loadConfig();loadDiagnostics();setInterval(load,1000);setInterval(loadDiagnostics,1000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    const rid_config_t *cfg = cfg_get();
    char body[512] = {0};
    char *cursor = body;
    size_t remaining = sizeof(body);
    bool first = true;
    appendf(&cursor, &remaining, "{");
    json_add_fmt(&cursor, &remaining, &first, "input_profile", "%u",
                 (unsigned)cfg->input_profile);
    json_add_string(&cursor, &remaining, &first, "input_profile_name",
                    input_profile_name(cfg->input_profile));
    json_add_string(&cursor, &remaining, &first, "protocol", "MAVLink2/ardupilotmega");
    json_add_fmt(&cursor, &remaining, &first, "baudrate", "%lu",
                 (unsigned long)cfg->uart_baud);
    json_add_fmt(&cursor, &remaining, &first, "mavlink_sysid", "%u",
                 (unsigned)cfg->mavlink_sysid);
    json_add_fmt(&cursor, &remaining, &first, "uas_type", "%u", (unsigned)cfg->ua_type);
    json_add_fmt(&cursor, &remaining, &first, "uas_id_type", "%u",
                 (unsigned)cfg->uas_id_type);
    json_add_string(&cursor, &remaining, &first, "uas_id", cfg->uas_id);
    appendf(&cursor,
            &remaining,
            ",\"reboot_required\":%s}",
            s_reboot_required ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    if (require_config_authorization(req) != ESP_OK) {
        return ESP_FAIL;
    }
    char body[CONFIG_BODY_MAX] = {0};
    if (receive_request_body(req, body, sizeof(body)) != ESP_OK) {
        return ESP_FAIL;
    }
    web_config_t config;
    if (!parse_form_config(body, &config) || !web_config_apply(&config)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid configuration");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req,
                              s_reboot_required
                                  ? "{\"ok\":true,\"reboot_required\":true}"
                                  : "{\"ok\":true,\"reboot_required\":false}");
}

static esp_err_t config_export_get_handler(httpd_req_t *req) {
    const rid_config_t *cfg = cfg_get();
    char body[384] = {0};
    snprintf(body,
             sizeof(body),
             "SERYRID_CONFIG_V1\n"
             "INPUT_PROFILE=%u\n"
             "BAUDRATE=%lu\n"
             "MAVLINK_SYSID=%u\n"
             "UAS_TYPE=%u\n"
             "UAS_ID_TYPE=%u\n"
             "UAS_ID=%s\n",
             (unsigned)cfg->input_profile,
             (unsigned long)cfg->uart_baud,
             (unsigned)cfg->mavlink_sysid,
             (unsigned)cfg->ua_type,
             (unsigned)cfg->uas_id_type,
             valid_uas_id(cfg->uas_id) ? cfg->uas_id : "");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req,
                       "Content-Disposition",
                       "attachment; filename=seryremoteid-config.txt");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t config_import_post_handler(httpd_req_t *req) {
    if (require_config_authorization(req) != ESP_OK) {
        return ESP_FAIL;
    }
    char body[CONFIG_BODY_MAX] = {0};
    if (receive_request_body(req, body, sizeof(body)) != ESP_OK) {
        return ESP_FAIL;
    }
    web_config_t config;
    if (!parse_import_config(body, &config) || !web_config_apply(&config)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid configuration file");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req,
                              s_reboot_required
                                  ? "{\"ok\":true,\"reboot_required\":true}"
                                  : "{\"ok\":true,\"reboot_required\":false}");
}

static esp_err_t diagnostics_get_handler(httpd_req_t *req) {
    rid_mavlink_diagnostics_t diagnostics;
    rid_mavlink_get_diagnostics(&diagnostics);
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t age = diagnostics.last_frame_ms == 0
                       ? UINT32_MAX
                       : now - diagnostics.last_frame_ms;
    char body[512] = {0};
    snprintf(body,
             sizeof(body),
             "{\"rx_bytes\":%lu,\"valid_frames\":%lu,\"parse_errors\":%lu,"
             "\"last_frame_age_ms\":%lu,\"last_message_id\":%lu,"
             "\"last_system_id\":%u,\"last_component_id\":%u}",
             (unsigned long)diagnostics.rx_bytes,
             (unsigned long)diagnostics.valid_frames,
             (unsigned long)diagnostics.parse_errors,
             (unsigned long)age,
             (unsigned long)diagnostics.last_message_id,
             (unsigned)diagnostics.last_system_id,
             (unsigned)diagnostics.last_component_id);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t reboot_post_handler(httpd_req_t *req) {
    if (require_config_authorization(req) != ESP_OK) {
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OK");
    ESP_LOGW(TAG, "configuration reboot requested");
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    char arm_reason[96] = {0};
    char status_reason[128] = {0};
    bool arm_ok = rid_state_arm_status(arm_reason, sizeof(arm_reason));
    ODID_UAS_Data uas;
    (void)rid_state_build_uas_data(&uas, status_reason, sizeof(status_reason));

    const esp_app_desc_t *desc = esp_app_get_description();
    const rid_config_t *cfg = cfg_get();
    char *body = calloc(1, 4096);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }

    char *cursor = body;
    size_t remaining = 4096;
    bool first = true;
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t sec = uptime_s % 60U;
    uint32_t min = (uptime_s / 60U) % 60U;
    uint32_t hr = (uptime_s / 3600U) % 24U;
    char tmp[24] = {0};

    appendf(&cursor, &remaining, "{");
    json_add_string(&cursor, &remaining, &first, "app", desc ? desc->project_name : "sery_remoteid");
    json_add_string(&cursor, &remaining, &first, "version", desc ? desc->version : "unknown");
    json_add_string(&cursor, &remaining, &first, "arm_ok", arm_ok ? "true" : "false");
    json_add_string(&cursor, &remaining, &first, "reason", arm_reason);
    json_add_string(&cursor, &remaining, &first, "wifi_ssid", cfg->wifi_ssid);
    json_add_fmt(&cursor, &remaining, &first, "wifi_channel", "%u", (unsigned)cfg->wifi_channel);
    json_add_fmt(&cursor, &remaining, &first, "mavlink_sysid", "%u",
                 (unsigned)rid_mavlink_system_id());

    json_add_string(&cursor, &remaining, &first, "STATUS:VERSION", desc ? desc->version : "unknown");
    json_add_fmt(&cursor, &remaining, &first, "STATUS:BOARD_ID", "%u", SERY_RID_CAN_BOARD_ID);
    json_add_fmt(&cursor, &remaining, &first, "STATUS:UPTIME", "%02lu:%02lu:%02lu",
                 (unsigned long)hr,
                 (unsigned long)min,
                 (unsigned long)sec);
    json_add_fmt(&cursor, &remaining, &first, "STATUS:FREEMEM", "%lu",
                 (unsigned long)esp_get_free_heap_size());

    json_add_enum(&cursor, &remaining, &first, "BASICID:UAType",
                  ENUM_UATYPE, ARRAY_LEN(ENUM_UATYPE), uas.BasicID[0].UAType);
    json_add_enum(&cursor, &remaining, &first, "BASICID:IDType",
                  ENUM_IDTYPE, ARRAY_LEN(ENUM_IDTYPE), uas.BasicID[0].IDType);
    json_add_string(&cursor, &remaining, &first, "BASICID:UASID", uas.BasicID[0].UASID);
    json_add_enum(&cursor, &remaining, &first, "BASICID:UAType2",
                  ENUM_UATYPE, ARRAY_LEN(ENUM_UATYPE), uas.BasicID[1].UAType);
    json_add_enum(&cursor, &remaining, &first, "BASICID:IDType2",
                  ENUM_IDTYPE, ARRAY_LEN(ENUM_IDTYPE), uas.BasicID[1].IDType);
    json_add_string(&cursor, &remaining, &first, "BASICID:UASID2", uas.BasicID[1].UASID);

    json_add_fmt(&cursor, &remaining, &first, "OPERATORID:IDType", "%u",
                 (unsigned)uas.OperatorID.OperatorIdType);
    json_add_string(&cursor, &remaining, &first, "OPERATORID:ID", uas.OperatorID.OperatorId);
    json_add_enum(&cursor, &remaining, &first, "SELFID:DescType",
                  ENUM_DESCTYPE, ARRAY_LEN(ENUM_DESCTYPE), uas.SelfID.DescType);
    json_add_string(&cursor, &remaining, &first, "SELFID:Desc", uas.SelfID.Desc);

    json_add_enum(&cursor, &remaining, &first, "SYSTEM:OperatorLocationType",
                  ENUM_LOCTYPE, ARRAY_LEN(ENUM_LOCTYPE), uas.System.OperatorLocationType);
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:ClassificationType",
                  ENUM_CLASSIF, ARRAY_LEN(ENUM_CLASSIF), uas.System.ClassificationType);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorLatitude",
                    latlon_string(uas.System.OperatorLatitude, uas.System.OperatorLongitude, true, tmp));
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorLongitude",
                    latlon_string(uas.System.OperatorLatitude, uas.System.OperatorLongitude, false, tmp));
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:AreaCount", "%u",
                 (unsigned)uas.System.AreaCount);
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:AreaRadius", "%u",
                 (unsigned)uas.System.AreaRadius);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:AreaCeiling",
                    alt_string(uas.System.AreaCeiling, tmp));
    json_add_string(&cursor, &remaining, &first, "SYSTEM:AreaFloor",
                    alt_string(uas.System.AreaFloor, tmp));
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:CategoryEU",
                  ENUM_CATEU, ARRAY_LEN(ENUM_CATEU), uas.System.CategoryEU);
    json_add_enum(&cursor, &remaining, &first, "SYSTEM:ClassEU",
                  ENUM_CLASSEU, ARRAY_LEN(ENUM_CLASSEU), uas.System.ClassEU);
    json_add_string(&cursor, &remaining, &first, "SYSTEM:OperatorAltitudeGeo",
                    alt_string(uas.System.OperatorAltitudeGeo, tmp));
    json_add_fmt(&cursor, &remaining, &first, "SYSTEM:Timestamp", "%u",
                 (unsigned)uas.System.Timestamp);

    json_add_enum(&cursor, &remaining, &first, "LOCATION:Status",
                  ENUM_STATUS, ARRAY_LEN(ENUM_STATUS), uas.Location.Status);
    json_add_string(&cursor, &remaining, &first, "LOCATION:StatusReason",
                    status_reason[0] ? status_reason : arm_reason);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:Direction", "%.2f",
                 (double)uas.Location.Direction);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:SpeedHorizontal", "%.2f",
                 (double)uas.Location.SpeedHorizontal);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:SpeedVertical", "%.2f",
                 (double)uas.Location.SpeedVertical);
    json_add_string(&cursor, &remaining, &first, "LOCATION:Latitude",
                    latlon_string(uas.Location.Latitude, uas.Location.Longitude, true, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:Longitude",
                    latlon_string(uas.Location.Latitude, uas.Location.Longitude, false, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:AltitudeBaro",
                    alt_string(uas.Location.AltitudeBaro, tmp));
    json_add_string(&cursor, &remaining, &first, "LOCATION:AltitudeGeo",
                    alt_string(uas.Location.AltitudeGeo, tmp));
    json_add_enum(&cursor, &remaining, &first, "LOCATION:HeightType",
                  ENUM_HEIGHT, ARRAY_LEN(ENUM_HEIGHT), uas.Location.HeightType);
    json_add_string(&cursor, &remaining, &first, "LOCATION:Height",
                    alt_string(uas.Location.Height, tmp));
    json_add_enum(&cursor, &remaining, &first, "LOCATION:HorizAccuracy",
                  ENUM_HACC, ARRAY_LEN(ENUM_HACC), uas.Location.HorizAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:VertAccuracy",
                  ENUM_VACC, ARRAY_LEN(ENUM_VACC), uas.Location.VertAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:BaroAccuracy",
                  ENUM_VACC, ARRAY_LEN(ENUM_VACC), uas.Location.BaroAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:SpeedAccuracy",
                  ENUM_SACC, ARRAY_LEN(ENUM_SACC), uas.Location.SpeedAccuracy);
    json_add_enum(&cursor, &remaining, &first, "LOCATION:TSAccuracy",
                  ENUM_TSACC, ARRAY_LEN(ENUM_TSACC), uas.Location.TSAccuracy);
    json_add_fmt(&cursor, &remaining, &first, "LOCATION:TimeStamp", "%.2f",
                 (double)uas.Location.TimeStamp);
    appendf(&cursor, &remaining, "}");

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return ret;
}

static esp_err_t favicon_get_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, NULL, 0);
}

static const char *ota_state_name(rid_ota_state_t state) {
    switch (state) {
    case RID_OTA_READY:
        return "ready";
    case RID_OTA_UPLOADING:
        return "uploading";
    case RID_OTA_VERIFYING:
        return "verifying";
    case RID_OTA_SUCCESS:
        return "success";
    case RID_OTA_ERROR:
        return "error";
    case RID_OTA_DISABLED:
    default:
        return "disabled";
    }
}

static void ota_set_state(rid_ota_state_t state, uint8_t progress, const char *error) {
    s_ota_state = state;
    s_ota_progress = progress > 100 ? 100 : progress;
    strlcpy(s_ota_error, error ? error : "", sizeof(s_ota_error));
}

static esp_err_t ota_send_text(httpd_req_t *req, const char *status, const char *text) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, text);
}

static esp_err_t ota_status_get_handler(httpd_req_t *req) {
    const esp_app_desc_t *desc = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    char body[768];
    snprintf(body,
             sizeof(body),
             "{\"enabled\":%s,\"active\":%s,\"state\":\"%s\",\"progress\":%u,"
             "\"error\":\"%s\",\"project\":\"%s\",\"version\":\"%s\","
             "\"build_date\":\"%s\",\"build_time\":\"%s\",\"idf\":\"%s\","
             "\"running_partition\":\"%s\",\"target_partition\":\"%s\","
             "\"target_size\":%lu}",
             s_ota_enabled ? "true" : "false",
             s_ota_active ? "true" : "false",
             ota_state_name(s_ota_state),
             (unsigned)s_ota_progress,
             s_ota_error,
             desc ? desc->project_name : "sery_remoteid",
             desc ? desc->version : "unknown",
             desc ? desc->date : "unknown",
             desc ? desc->time : "unknown",
             desc ? desc->idf_ver : "unknown",
             running ? running->label : "unknown",
             target ? target->label : "",
             target ? (unsigned long)target->size : 0UL);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t ota_enable_post_handler(httpd_req_t *req) {
    if (require_config_authorization(req) != ESP_OK) {
        return ESP_FAIL;
    }
    if (s_ota_active) {
        return ota_send_text(req, "409 Conflict", "OTA upload already active");
    }
    if (!esp_ota_get_next_update_partition(NULL)) {
        ota_set_state(RID_OTA_ERROR, 0, "no inactive OTA partition");
        return ota_send_text(req, "500 Internal Server Error", "no inactive OTA partition");
    }
    s_ota_enabled = true;
    ota_set_state(RID_OTA_READY, 0, NULL);
    ESP_LOGI(TAG, "OTA enabled until reboot or upload completion");
    return ota_send_text(req, "200 OK", "OTA enabled");
}

static esp_err_t ota_page_get_handler(httpd_req_t *req) {
    static const char html[] =
        "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>SeryRemoteID OTA</title><style>"
        ":root{--bg:#071018;--card:#0b1923;--ink:#edf7ff;--muted:#9ab2c2;--line:#1d465b;--blue:#16a4d8;--green:#17a668;--red:#d84b55}"
        "*{box-sizing:border-box}body{margin:0;min-height:100vh;padding:22px;background:radial-gradient(circle at top,#10354a 0,#071018 46%);color:var(--ink);font-family:system-ui,-apple-system,Segoe UI,sans-serif}"
        ".card{max-width:680px;margin:0 auto;padding:22px;border:1px solid var(--line);border-radius:18px;background:rgba(9,20,29,.96);box-shadow:0 18px 50px rgba(0,0,0,.35)}"
        "h1{margin:0 0 6px;font-size:26px}.sub,.hint{color:var(--muted);font-size:13px;line-height:1.5}.status{display:flex;justify-content:space-between;gap:12px;align-items:center;margin:18px 0;padding:14px;border:1px solid var(--line);border-radius:12px;background:#08141d}"
        ".badge{padding:5px 10px;border-radius:999px;background:#343f48;color:#fff;font-size:12px;font-weight:700}.badge.ready{background:var(--blue)}.badge.uploading,.badge.verifying{background:#a86b00}.badge.success{background:var(--green)}.badge.error{background:var(--red)}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:14px 0}.kv{padding:10px;border:1px solid var(--line);border-radius:10px}.kv span{display:block;color:var(--muted);font-size:11px}.kv b{display:block;margin-top:4px;font-size:13px;overflow-wrap:anywhere}"
        ".box{margin-top:14px;padding:14px;border:1px solid var(--line);border-radius:12px}label{display:block;margin-bottom:6px;color:var(--muted);font-size:12px}input[type=password],input[type=file]{width:100%;padding:10px;border:1px solid #2a566c;border-radius:8px;background:#061018;color:var(--ink)}"
        ".actions{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-top:10px}button,a.btn{border:1px solid #2b8eb3;border-radius:8px;padding:9px 14px;background:#087aa7;color:#fff;font-weight:700;text-decoration:none;cursor:pointer}button:disabled{opacity:.45;cursor:not-allowed}progress{width:100%;height:12px;margin-top:12px}#msg{min-height:20px;margin:10px 0 0;color:var(--muted);font-size:13px}"
        "@media(max-width:560px){body{padding:12px}.card{padding:16px}.grid{grid-template-columns:1fr}}"
        "</style></head><body><main class=\"card\"><h1>Firmware Update</h1>"
        "<div class=\"sub\">OTA 默认关闭。输入设备配置口令开启本次维护窗口；重启或升级成功后自动关闭。</div>"
        "<div class=\"status\"><div><b id=\"stateText\">读取状态...</b><div id=\"stateHint\" class=\"hint\"></div></div><span id=\"badge\" class=\"badge\">LOCKED</span></div>"
        "<div class=\"grid\"><div class=\"kv\"><span>当前固件</span><b id=\"current\">-</b></div><div class=\"kv\"><span>构建环境</span><b id=\"build\">-</b></div><div class=\"kv\"><span>运行分区</span><b id=\"running\">-</b></div><div class=\"kv\"><span>目标分区 / 最大镜像</span><b id=\"target\">-</b></div></div>"
        "<section class=\"box\"><label for=\"key\">配置口令</label><input id=\"key\" type=\"password\" maxlength=\"20\" autocomplete=\"current-password\"><div class=\"actions\"><button id=\"enable\" type=\"button\">开启 OTA</button><a class=\"btn\" href=\"/\">返回状态页</a></div></section>"
        "<section class=\"box\"><label for=\"fw\">选择 SeryRemoteID app .bin</label><input id=\"fw\" type=\"file\" accept=\".bin,application/octet-stream\"><div class=\"actions\"><button id=\"upload\" type=\"button\" disabled>上传、校验并重启</button></div><progress id=\"bar\" value=\"0\" max=\"100\"></progress><p id=\"msg\"></p><div class=\"hint\">上传前不会停止 AP 或 Remote ID 广播。设备端会校验 ESP 镜像、Board ID 和现有签名策略；校验失败不会切换启动分区。</div></section>"
        "</main><script>"
        "const $=id=>document.getElementById(id);let info=null,uploading=false;"
        "const fmt=n=>n?((n/1024/1024).toFixed(2)+' MiB'):'不可用';"
        "function render(d){info=d;$('badge').className='badge '+d.state;$('badge').textContent=d.state.toUpperCase();$('stateText').textContent=d.enabled?'OTA 维护窗口已开启':'OTA 维护窗口已关闭';$('stateHint').textContent=d.error||('设备进度 '+d.progress+'%');$('current').textContent=d.project+' '+d.version;$('build').textContent=d.build_date+' '+d.build_time+' / '+d.idf;$('running').textContent=d.running_partition;$('target').textContent=(d.target_partition||'无')+' / '+fmt(d.target_size);$('enable').disabled=d.enabled||d.active||uploading;validateFile();}"
        "async function load(){if(uploading)return;try{const r=await fetch('/api/ota/status',{cache:'no-store'});render(await r.json());}catch(e){$('msg').textContent='读取 OTA 状态失败: '+e;}}"
        "function validateFile(){const f=$('fw').files[0];let problem='';if(f&&info&&info.target_size&&f.size>info.target_size)problem='镜像过大：'+fmt(f.size)+' > '+fmt(info.target_size);$('msg').textContent=problem;$('upload').disabled=!f||!info||!info.enabled||info.active||!!problem||uploading;}"
        "async function enable(){const key=$('key').value;if(!key){$('msg').textContent='请输入配置口令';return;}$('enable').disabled=true;$('msg').textContent='正在开启 OTA...';try{const r=await fetch('/api/ota/enable',{method:'POST',headers:{'X-Sery-Config-Key':key}});const t=await r.text();if(!r.ok)throw new Error(t||('HTTP '+r.status));$('msg').textContent='OTA 已开启，请选择固件';await load();}catch(e){$('msg').textContent=String(e.message||e);$('enable').disabled=false;}}"
        "function upload(){const f=$('fw').files[0];validateFile();if($('upload').disabled||!f)return;if(!confirm('确认上传 '+f.name+' ('+fmt(f.size)+')？成功后设备会重启。'))return;uploading=true;$('upload').disabled=true;$('enable').disabled=true;$('bar').value=0;$('msg').textContent='上传中，请保持页面打开...';const x=new XMLHttpRequest();x.open('POST','/ota/upload');x.timeout=180000;x.setRequestHeader('Content-Type','application/octet-stream');x.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded*100/e.total);$('bar').value=p;$('msg').textContent='上传 '+p+'%';}};x.onload=()=>{const ok=x.status>=200&&x.status<300;$('msg').textContent=x.responseText||('HTTP '+x.status);if(ok){$('bar').value=100;$('badge').className='badge success';$('badge').textContent='REBOOT';}else{uploading=false;load();}};x.onerror=()=>{$('msg').textContent='连接中断；如果设备已重启，请等待后重新打开页面。';uploading=false;setTimeout(load,2000);};x.ontimeout=()=>{$('msg').textContent='上传超时，当前固件保持不变。';uploading=false;load();};x.send(f);}"
        "$('enable').addEventListener('click',enable);$('upload').addEventListener('click',upload);$('fw').addEventListener('change',validateFile);load();setInterval(load,2000);"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_upload_fail(httpd_req_t *req,
                                 ota_upload_t *ota,
                                 const char *status,
                                 const char *message) {
    if (ota && ota->started) {
        esp_ota_abort(ota->handle);
        ota->started = false;
    }
    s_ota_active = false;
    ota_set_state(RID_OTA_ERROR, s_ota_progress, message);
    ESP_LOGE(TAG,
             "OTA failed after %u bytes: %s (%s)",
             ota ? (unsigned)ota->written : 0U,
             message,
             ota ? esp_err_to_name(ota->err) : "n/a");
    return ota_send_text(req, status, message);
}

static esp_err_t update_post_handler(httpd_req_t *req) {
    if (!s_ota_enabled) {
        return ota_send_text(req, "403 Forbidden", "OTA disabled. Open /ota and enable it first.");
    }
    if (s_ota_active) {
        return ota_send_text(req, "409 Conflict", "OTA upload already active");
    }

    ota_upload_t ota = {
        .part = esp_ota_get_next_update_partition(NULL),
        .handle = 0,
        .started = false,
        .err = ESP_OK,
        .written = 0,
    };
    if (!ota.part) {
        ota.err = ESP_ERR_NOT_FOUND;
        return ota_upload_fail(req, &ota, "500 Internal Server Error", "no OTA partition");
    }
    if (req->content_len == 0) {
        ota.err = ESP_ERR_INVALID_SIZE;
        return ota_upload_fail(req, &ota, "411 Length Required", "Content-Length required");
    }
    if ((size_t)req->content_len > ota.part->size) {
        ota.err = ESP_ERR_INVALID_SIZE;
        return ota_upload_fail(req, &ota, "413 Payload Too Large", "firmware image too large");
    }

    ESP_LOGI(TAG,
             "OTA upload start: partition=%s size=%u",
             ota.part->label,
             (unsigned)req->content_len);
    s_ota_active = true;
    ota_set_state(RID_OTA_UPLOADING, 0, NULL);
    ota.err = esp_ota_begin(ota.part, req->content_len, &ota.handle);
    if (ota.err != ESP_OK) {
        return ota_upload_fail(req, &ota, "500 Internal Server Error", "esp_ota_begin failed");
    }
    ota.started = true;

    int remaining = req->content_len;
    uint8_t last_log_bucket = 255;
    while (remaining > 0) {
        int recv = httpd_req_recv(req,
                                  s_ota_chunk,
                                  remaining > (int)sizeof(s_ota_chunk)
                                      ? sizeof(s_ota_chunk)
                                      : (size_t)remaining);
        if (recv <= 0) {
            ota.err = ESP_FAIL;
            return ota_upload_fail(req, &ota, "400 Bad Request", "upload interrupted");
        }
        ota.err = esp_ota_write(ota.handle, s_ota_chunk, (size_t)recv);
        if (ota.err != ESP_OK) {
            return ota_upload_fail(req, &ota, "500 Internal Server Error", "esp_ota_write failed");
        }
        ota.written += (size_t)recv;
        remaining -= recv;
        uint8_t progress = (uint8_t)((ota.written * 99U) / (size_t)req->content_len);
        ota_set_state(RID_OTA_UPLOADING, progress, NULL);
        uint8_t bucket = progress / 10U;
        if (progress > 0 && bucket != last_log_bucket) {
            last_log_bucket = bucket;
            ESP_LOGI(TAG, "OTA progress: %u%%", (unsigned)progress);
        }
        vTaskDelay(1);
    }

    ota_set_state(RID_OTA_VERIFYING, 99, NULL);
    ota.err = esp_ota_end(ota.handle);
    ota.started = false;
    if (ota.err != ESP_OK) {
        return ota_upload_fail(req, &ota, "400 Bad Request", "invalid ESP firmware image");
    }
    if (!rid_firmware_check_ota_partition(ota.part)) {
        ota.err = ESP_ERR_INVALID_CRC;
        return ota_upload_fail(req,
                               &ota,
                               "400 Bad Request",
                               "firmware compatibility or signature check failed");
    }
    ota.err = esp_ota_set_boot_partition(ota.part);
    if (ota.err != ESP_OK) {
        return ota_upload_fail(req,
                               &ota,
                               "500 Internal Server Error",
                               "esp_ota_set_boot_partition failed");
    }

    s_ota_enabled = false;
    s_ota_active = false;
    ota_set_state(RID_OTA_SUCCESS, 100, NULL);
    ESP_LOGW(TAG,
             "OTA upload complete: partition=%s bytes=%u; rebooting",
             ota.part->label,
             (unsigned)ota.written);
    esp_err_t response_err =
        ota_send_text(req, "200 OK", "Upload verified. SeryRemoteID will reboot in 2 seconds.");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return response_err;
}

esp_err_t rid_web_start(void) {
    if (!cfg_get()->webserver_enable) {
        ESP_LOGI(TAG, "web server disabled");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 4;
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "httpd_start failed");

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t status = {
        .uri = "/ajax/status.json",
        .method = HTTP_GET,
        .handler = status_get_handler,
    };
    const httpd_uri_t update = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = update_post_handler,
    };
    const httpd_uri_t ota_page = {
        .uri = "/ota",
        .method = HTTP_GET,
        .handler = ota_page_get_handler,
    };
    const httpd_uri_t ota_upload = {
        .uri = "/ota/upload",
        .method = HTTP_POST,
        .handler = update_post_handler,
    };
    const httpd_uri_t ota_status = {
        .uri = "/api/ota/status",
        .method = HTTP_GET,
        .handler = ota_status_get_handler,
    };
    const httpd_uri_t ota_enable = {
        .uri = "/api/ota/enable",
        .method = HTTP_POST,
        .handler = ota_enable_post_handler,
    };
    const httpd_uri_t favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_get_handler,
    };
    const httpd_uri_t config_get = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
    };
    const httpd_uri_t config_post = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
    };
    const httpd_uri_t config_export = {
        .uri = "/api/config/export",
        .method = HTTP_GET,
        .handler = config_export_get_handler,
    };
    const httpd_uri_t config_import = {
        .uri = "/api/config/import",
        .method = HTTP_POST,
        .handler = config_import_post_handler,
    };
    const httpd_uri_t diagnostics = {
        .uri = "/api/input/diagnostics",
        .method = HTTP_GET,
        .handler = diagnostics_get_handler,
    };
    const httpd_uri_t reboot = {
        .uri = "/api/reboot",
        .method = HTTP_POST,
        .handler = reboot_post_handler,
    };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &status);
    httpd_register_uri_handler(s_server, &update);
    httpd_register_uri_handler(s_server, &ota_page);
    httpd_register_uri_handler(s_server, &ota_upload);
    httpd_register_uri_handler(s_server, &ota_status);
    httpd_register_uri_handler(s_server, &ota_enable);
    httpd_register_uri_handler(s_server, &favicon);
    httpd_register_uri_handler(s_server, &config_get);
    httpd_register_uri_handler(s_server, &config_post);
    httpd_register_uri_handler(s_server, &config_export);
    httpd_register_uri_handler(s_server, &config_import);
    httpd_register_uri_handler(s_server, &diagnostics);
    httpd_register_uri_handler(s_server, &reboot);
    ESP_LOGI(TAG, "web server started");
    return ESP_OK;
}
