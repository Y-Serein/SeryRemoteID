#include "protocol/rid_state.h"

#include <stdio.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "rid_state";

typedef struct {
    mavlink_open_drone_id_basic_id_t basic_id;
    mavlink_open_drone_id_location_t location;
    mavlink_open_drone_id_authentication_t auth[ODID_AUTH_MAX_PAGES];
    mavlink_open_drone_id_self_id_t self_id;
    mavlink_open_drone_id_system_t system;
    mavlink_open_drone_id_operator_id_t operator_id;

    uint32_t last_basic_id_ms;
    uint32_t last_location_ms;
    uint32_t last_self_id_ms;
    uint32_t last_system_ms;
    uint32_t last_operator_id_ms;
    uint32_t last_auth_ms[ODID_AUTH_MAX_PAGES];
    uint32_t last_system_timestamp;
    float last_location_timestamp;
} rid_state_t;

static rid_state_t s_state;
static SemaphoreHandle_t s_lock;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void append_reason(char *reason, size_t reason_size, const char *token) {
    if (!reason || reason_size == 0 || !token) {
        return;
    }
    size_t len = strnlen(reason, reason_size);
    if (len >= reason_size - 1) {
        return;
    }
    snprintf(&reason[len], reason_size - len, "%s", token);
}

static bool copy_mav_char_field(char *dst, size_t dst_size,
                                const void *src, size_t src_size) {
    const uint8_t *in = (const uint8_t *)src;
    if (!dst || dst_size == 0 || !src) {
        return false;
    }
    memset(dst, 0, dst_size);
    size_t i = 0;
    for (; i < src_size && i < dst_size - 1; i++) {
        if (in[i] == 0) {
            break;
        }
        dst[i] = (char)in[i];
    }
    return i > 0;
}

static bool basic_id_is_valid(const mavlink_open_drone_id_basic_id_t *msg) {
    if (!msg || msg->ua_type == 0 || msg->id_type == 0) {
        return false;
    }
    for (size_t i = 0; i < sizeof(msg->uas_id); i++) {
        if (msg->uas_id[i] != 0) {
            return true;
        }
    }
    return false;
}

static bool mav_basic_id_differs_from_cfg(const mavlink_open_drone_id_basic_id_t *msg,
                                          const rid_config_t *cfg) {
    char uas_id[21] = {0};
    copy_mav_char_field(uas_id, sizeof(uas_id), msg->uas_id, sizeof(msg->uas_id));
    return msg->ua_type != cfg->ua_type ||
           msg->id_type != cfg->uas_id_type ||
           strcmp(uas_id, cfg->uas_id) != 0;
}

void rid_state_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);
    ESP_LOGI(TAG, "RemoteID state initialised");
}

void rid_state_update_basic_id(const mavlink_open_drone_id_basic_id_t *msg) {
    if (!msg || !basic_id_is_valid(msg)) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.basic_id = *msg;
    s_state.last_basic_id_ms = now_ms();
    xSemaphoreGive(s_lock);

    const rid_config_t *cfg = cfg_get();
    if (!cfg_have_basic_id() &&
        !(cfg->options & RID_OPTIONS_DONT_SAVE_BASIC_ID_TO_PARAMS)) {
        char uas_id[21] = {0};
        copy_mav_char_field(uas_id, sizeof(uas_id), msg->uas_id, sizeof(msg->uas_id));
        cfg_param_set_by_name_uint8("UAS_TYPE", msg->ua_type);
        cfg_param_set_by_name_uint8("UAS_ID_TYPE", msg->id_type);
        cfg_param_set_by_name_string("UAS_ID", uas_id);
        ESP_LOGI(TAG, "persisted BasicID from MAVLink");
    }
}

void rid_state_update_location(const mavlink_open_drone_id_location_t *msg) {
    if (!msg) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.location = *msg;
    s_state.last_location_ms = now_ms();
    s_state.last_location_timestamp = msg->timestamp;
    xSemaphoreGive(s_lock);
}

void rid_state_update_authentication(const mavlink_open_drone_id_authentication_t *msg) {
    if (!msg || msg->data_page >= ODID_AUTH_MAX_PAGES) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.auth[msg->data_page] = *msg;
    s_state.last_auth_ms[msg->data_page] = now_ms();
    xSemaphoreGive(s_lock);
}

void rid_state_update_self_id(const mavlink_open_drone_id_self_id_t *msg) {
    if (!msg || msg->description[0] == 0) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.self_id = *msg;
    s_state.last_self_id_ms = now_ms();
    xSemaphoreGive(s_lock);
}

void rid_state_update_system(const mavlink_open_drone_id_system_t *msg) {
    if (!msg) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.system = *msg;
    s_state.last_system_ms = now_ms();
    s_state.last_system_timestamp = msg->timestamp;
    xSemaphoreGive(s_lock);
}

void rid_state_update_system_update(const mavlink_open_drone_id_system_update_t *msg) {
    if (!msg) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_state.last_system_ms != 0) {
        s_state.system.operator_latitude = msg->operator_latitude;
        s_state.system.operator_longitude = msg->operator_longitude;
        s_state.system.operator_altitude_geo = msg->operator_altitude_geo;
        s_state.system.timestamp = msg->timestamp;
        s_state.last_system_ms = now_ms();
        s_state.last_system_timestamp = msg->timestamp;
    }
    xSemaphoreGive(s_lock);
}

void rid_state_update_operator_id(const mavlink_open_drone_id_operator_id_t *msg) {
    if (!msg || msg->operator_id[0] == 0) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state.operator_id = *msg;
    s_state.last_operator_id_ms = now_ms();
    xSemaphoreGive(s_lock);
}

static void fill_basic_id_from_cfg_values(ODID_UAS_Data *out,
                                          uint8_t index,
                                          uint8_t ua_type,
                                          uint8_t id_type,
                                          const char *uas_id) {
    if (index >= ODID_BASIC_ID_MAX_MESSAGES || !uas_id) {
        return;
    }
    out->BasicID[index].UAType = (ODID_uatype_t)ua_type;
    out->BasicID[index].IDType = (ODID_idtype_t)id_type;
    strncpy(out->BasicID[index].UASID, uas_id, ODID_ID_SIZE);
    out->BasicIDValid[index] = 1;
}

static void fill_basic_id_from_mavlink(ODID_UAS_Data *out, uint8_t index,
                                       const mavlink_open_drone_id_basic_id_t *msg) {
    if (index >= ODID_BASIC_ID_MAX_MESSAGES) {
        return;
    }
    out->BasicID[index].UAType = (ODID_uatype_t)msg->ua_type;
    out->BasicID[index].IDType = (ODID_idtype_t)msg->id_type;
    copy_mav_char_field(out->BasicID[index].UASID,
                        sizeof(out->BasicID[index].UASID),
                        msg->uas_id,
                        sizeof(msg->uas_id));
    out->BasicIDValid[index] = 1;
}

static void fill_location(ODID_UAS_Data *out,
                          const mavlink_open_drone_id_location_t *msg) {
    out->Location.Status = (ODID_status_t)msg->status;
    out->Location.Direction = msg->direction * 0.01f;
    out->Location.SpeedHorizontal = msg->speed_horizontal * 0.01f;
    out->Location.SpeedVertical = msg->speed_vertical * 0.01f;
    out->Location.Latitude = msg->latitude * 1.0e-7;
    out->Location.Longitude = msg->longitude * 1.0e-7;
    out->Location.AltitudeBaro = msg->altitude_barometric;
    out->Location.AltitudeGeo = msg->altitude_geodetic;
    out->Location.HeightType = (ODID_Height_reference_t)msg->height_reference;
    out->Location.Height = msg->height;
    out->Location.HorizAccuracy = (ODID_Horizontal_accuracy_t)msg->horizontal_accuracy;
    out->Location.VertAccuracy = (ODID_Vertical_accuracy_t)msg->vertical_accuracy;
    out->Location.BaroAccuracy = (ODID_Vertical_accuracy_t)msg->barometer_accuracy;
    out->Location.SpeedAccuracy = (ODID_Speed_accuracy_t)msg->speed_accuracy;
    out->Location.TSAccuracy = (ODID_Timestamp_accuracy_t)msg->timestamp_accuracy;
    out->Location.TimeStamp = msg->timestamp;
    out->LocationValid = 1;
}

static void fill_system(ODID_UAS_Data *out,
                        const mavlink_open_drone_id_system_t *msg) {
    out->System.OperatorLocationType =
        (ODID_operator_location_type_t)msg->operator_location_type;
    out->System.ClassificationType =
        (ODID_classification_type_t)msg->classification_type;
    out->System.OperatorLatitude = msg->operator_latitude * 1.0e-7;
    out->System.OperatorLongitude = msg->operator_longitude * 1.0e-7;
    out->System.AreaCount = msg->area_count;
    out->System.AreaRadius = msg->area_radius;
    out->System.AreaCeiling = msg->area_ceiling;
    out->System.AreaFloor = msg->area_floor;
    out->System.CategoryEU = (ODID_category_EU_t)msg->category_eu;
    out->System.ClassEU = (ODID_class_EU_t)msg->class_eu;
    out->System.OperatorAltitudeGeo = msg->operator_altitude_geo;
    out->System.Timestamp = msg->timestamp;
    out->SystemValid = 1;
}

static bool validate_uas_data(ODID_UAS_Data *out, char *reason, size_t reason_size) {
    bool ok = true;

    if (out->LocationValid) {
        ODID_Location_encoded encoded = {0};
        if (encodeLocationMessage(&encoded, &out->Location) != ODID_SUCCESS) {
            append_reason(reason, reason_size, "bad LOC data ");
            ok = false;
        }
    }
    if (out->SystemValid) {
        ODID_System_encoded encoded = {0};
        if (encodeSystemMessage(&encoded, &out->System) != ODID_SUCCESS) {
            append_reason(reason, reason_size, "bad SYS data ");
            ok = false;
        }
    }
    for (uint8_t i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; i++) {
        if (out->BasicIDValid[i]) {
            ODID_BasicID_encoded encoded = {0};
            if (encodeBasicIDMessage(&encoded, &out->BasicID[i]) != ODID_SUCCESS) {
                append_reason(reason, reason_size, i == 0 ? "bad ID_1 data " : "bad ID_2 data ");
                ok = false;
            }
        }
    }
    if (out->SelfIDValid) {
        ODID_SelfID_encoded encoded = {0};
        if (encodeSelfIDMessage(&encoded, &out->SelfID) != ODID_SUCCESS) {
            append_reason(reason, reason_size, "bad SELF_ID data ");
            ok = false;
        }
    }
    if (out->OperatorIDValid) {
        ODID_OperatorID_encoded encoded = {0};
        if (encodeOperatorIDMessage(&encoded, &out->OperatorID) != ODID_SUCCESS) {
            append_reason(reason, reason_size, "bad OP_ID data ");
            ok = false;
        }
    }
    return ok;
}

bool rid_state_build_uas_data(ODID_UAS_Data *out, char *reason, size_t reason_size) {
    if (!out) {
        return false;
    }
    if (reason && reason_size > 0) {
        reason[0] = 0;
    }

    rid_state_t local;
    const uint32_t now = now_ms();
    const rid_config_t *cfg = cfg_get();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    local = s_state;
    xSemaphoreGive(s_lock);

    odid_initUasData(out);

    const bool mav_basic_valid = basic_id_is_valid(&local.basic_id);
    if (cfg->options & RID_OPTIONS_DONT_SAVE_BASIC_ID_TO_PARAMS) {
        if (mav_basic_valid) {
            fill_basic_id_from_mavlink(out, 0, &local.basic_id);
        } else if (cfg_have_basic_id()) {
            fill_basic_id_from_cfg_values(out, 0, cfg->ua_type, cfg->uas_id_type, cfg->uas_id);
        }
    } else if (cfg_have_basic_id()) {
        fill_basic_id_from_cfg_values(out, 0, cfg->ua_type, cfg->uas_id_type, cfg->uas_id);
        if (cfg_have_basic_id_2()) {
            fill_basic_id_from_cfg_values(out, 1, cfg->ua_type_2, cfg->uas_id_type_2, cfg->uas_id_2);
        } else if (mav_basic_valid && mav_basic_id_differs_from_cfg(&local.basic_id, cfg)) {
            fill_basic_id_from_mavlink(out, 1, &local.basic_id);
        }
    } else if (mav_basic_valid) {
        fill_basic_id_from_mavlink(out, 0, &local.basic_id);
    } else {
        append_reason(reason, reason_size, "ID ");
    }

    if (local.last_location_ms != 0) {
        fill_location(out, &local.location);
        if (now - local.last_location_ms > SERY_RID_LOCATION_STALE_MS) {
            out->Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
            append_reason(reason, reason_size, "LOC ");
        }
    } else if (cfg->broadcast_powerup) {
        out->Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
        out->LocationValid = 1;
        append_reason(reason, reason_size, "LOC ");
    } else {
        append_reason(reason, reason_size, "LOC ");
        return false;
    }

    if (local.last_system_ms != 0) {
        fill_system(out, &local.system);
        if (now - local.last_system_ms > SERY_RID_SYSTEM_STALE_MS) {
            out->Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
            append_reason(reason, reason_size, "SYS ");
        }
    } else {
        append_reason(reason, reason_size, "SYS ");
    }

    if (local.last_self_id_ms != 0 && now - local.last_self_id_ms <= SERY_RID_AUX_STALE_MS) {
        out->SelfID.DescType = (ODID_desctype_t)local.self_id.description_type;
        copy_mav_char_field(out->SelfID.Desc, sizeof(out->SelfID.Desc),
                            local.self_id.description, sizeof(local.self_id.description));
        out->SelfIDValid = 1;
    }

    if (local.last_operator_id_ms != 0 &&
        now - local.last_operator_id_ms <= SERY_RID_AUX_STALE_MS) {
        out->OperatorID.OperatorIdType =
            (ODID_operatorIdType_t)local.operator_id.operator_id_type;
        copy_mav_char_field(out->OperatorID.OperatorId, sizeof(out->OperatorID.OperatorId),
                            local.operator_id.operator_id,
                            sizeof(local.operator_id.operator_id));
        out->OperatorIDValid = 1;
    }

    for (uint8_t i = 0; i < ODID_AUTH_MAX_PAGES; i++) {
        if (local.last_auth_ms[i] == 0 || now - local.last_auth_ms[i] > SERY_RID_AUX_STALE_MS) {
            continue;
        }
        const mavlink_open_drone_id_authentication_t *auth = &local.auth[i];
        out->Auth[i].DataPage = auth->data_page;
        out->Auth[i].AuthType = (ODID_authtype_t)auth->authentication_type;
        out->Auth[i].LastPageIndex = auth->last_page_index;
        out->Auth[i].Length = auth->length;
        out->Auth[i].Timestamp = auth->timestamp;
        memcpy(out->Auth[i].AuthData,
               auth->authentication_data,
               sizeof(auth->authentication_data));
        out->AuthValid[i] = 1;
    }

    if (!validate_uas_data(out, reason, reason_size)) {
        out->Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
        return false;
    }

    return out->BasicIDValid[0] && out->LocationValid;
}

bool rid_state_arm_status(char *reason, size_t reason_size) {
    if (reason && reason_size > 0) {
        reason[0] = 0;
    }

    rid_state_t local;
    uint32_t now = now_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    local = s_state;
    xSemaphoreGive(s_lock);

    if (cfg_get()->options & RID_OPTIONS_FORCE_ARM_OK) {
        return true;
    }

    if (local.last_location_ms == 0 || now - local.last_location_ms > SERY_RID_LOCATION_STALE_MS) {
        append_reason(reason, reason_size, "LOC ");
    }
    if (!cfg_have_basic_id() && !basic_id_is_valid(&local.basic_id)) {
        append_reason(reason, reason_size, "ID ");
    }
    if (local.last_self_id_ms == 0 || now - local.last_self_id_ms > SERY_RID_AUX_STALE_MS) {
        append_reason(reason, reason_size, "SELF_ID ");
    }
    if (local.last_operator_id_ms == 0 || now - local.last_operator_id_ms > SERY_RID_AUX_STALE_MS) {
        append_reason(reason, reason_size, "OP_ID ");
    }
    if (local.last_system_ms == 0 || now - local.last_system_ms > SERY_RID_SYSTEM_STALE_MS) {
        append_reason(reason, reason_size, "SYS ");
    }
    if (local.location.latitude == 0 && local.location.longitude == 0) {
        append_reason(reason, reason_size, "LOC ");
    }
    if (local.system.operator_latitude == 0 && local.system.operator_longitude == 0) {
        append_reason(reason, reason_size, "OP_LOC ");
    }

    return !reason || reason_size == 0 || reason[0] == 0;
}

bool rid_state_have_location(void) {
    bool ret;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    ret = s_state.last_location_ms != 0;
    xSemaphoreGive(s_lock);
    return ret;
}
