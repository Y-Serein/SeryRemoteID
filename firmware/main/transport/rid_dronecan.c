#include "transport/rid_dronecan.h"

#include <stdio.h>
#include <string.h>

#include "canard.h"
#include "cfg.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "dronecan.remoteid.ArmStatus.h"
#include "dronecan.remoteid.BasicID.h"
#include "dronecan.remoteid.Location.h"
#include "dronecan.remoteid.OperatorID.h"
#include "dronecan.remoteid.SecureCommand.h"
#include "dronecan.remoteid.SelfID.h"
#include "dronecan.remoteid.System.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "protocol/rid_state.h"
#include "security/rid_secure.h"
#include "uavcan.protocol.GetNodeInfo.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.RestartNode.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"
#include "uavcan.protocol.param.GetSet.h"

static const char *TAG = "rid_dronecan";

#define RID_CAN_POOL_SIZE 4096

static CanardInstance s_canard;
static uint32_t s_canard_pool[RID_CAN_POOL_SIZE / sizeof(uint32_t)];
static StackType_t s_dronecan_stack[SERY_RID_TASK_STACK_WORDS];
static StaticTask_t s_dronecan_tcb;

static uint8_t s_node_status_tid;
static uint8_t s_arm_status_tid;
static uint8_t s_dna_tid;
static uint32_t s_last_node_status_ms;
static uint32_t s_last_cleanup_ms;
static uint32_t s_next_node_id_allocation_request_ms;
static uint32_t s_node_id_allocation_unique_id_offset;
static uint8_t s_tx_fail_count;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static uint64_t now_us(void) {
    return (uint64_t)esp_timer_get_time();
}

static void copy_bytes(uint8_t *dst, size_t dst_size, const uint8_t *src, size_t src_len) {
    if (!dst || dst_size == 0) {
        return;
    }
    memset(dst, 0, dst_size);
    if (!src) {
        return;
    }
    if (src_len > dst_size) {
        src_len = dst_size;
    }
    memcpy(dst, src, src_len);
}

static void read_unique_id(uint8_t id[16]) {
    memset(id, 0, 16);
    esp_efuse_mac_get_default(id);
}

static void configure_termination(void) {
#if SERY_RID_CAN_TERM_GPIO >= 0
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SERY_RID_CAN_TERM_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));
    gpio_set_level(SERY_RID_CAN_TERM_GPIO,
                   cfg_get()->can_terminate ? SERY_RID_CAN_TERM_ON_LEVEL
                                            : !SERY_RID_CAN_TERM_ON_LEVEL);
#endif
}

static esp_err_t can_driver_start(void) {
    if (SERY_RID_CAN_TX_GPIO < 0 || SERY_RID_CAN_RX_GPIO < 0) {
        ESP_LOGW(TAG, "DroneCAN disabled: CAN pins are not configured");
        return ESP_ERR_NOT_SUPPORTED;
    }

    configure_termination();

    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter = {
        .acceptance_code = 0x10000000U << 3,
        .acceptance_mask = 0x0FFFFFFFU << 3,
        .single_filter = true,
    };
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SERY_RID_CAN_TX_GPIO,
                                    (gpio_num_t)SERY_RID_CAN_RX_GPIO,
                                    TWAI_MODE_NORMAL);
    general.tx_queue_len = 5;
    general.rx_queue_len = 50;
    general.alerts_enabled = TWAI_ALERT_NONE;

    esp_err_t err = twai_driver_install(&general, &timing, &filter);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    const uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
    ESP_RETURN_ON_ERROR(twai_reconfigure_alerts(alerts, NULL),
                        TAG,
                        "twai_reconfigure_alerts failed");

    err = twai_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    ESP_LOGI(TAG,
             "DroneCAN TWAI started: tx=%d rx=%d bitrate=%d node=%u",
             SERY_RID_CAN_TX_GPIO,
             SERY_RID_CAN_RX_GPIO,
             SERY_RID_CAN_BITRATE,
             (unsigned)cfg_get()->can_node);
    return ESP_OK;
}

static bool can_send_frame(const CanardCANFrame *frame) {
    if (!frame || frame->data_len > 8 || (frame->id & CANARD_CAN_FRAME_ERR)) {
        return false;
    }

    twai_status_info_t info = {0};
    if (twai_get_status_info(&info) == ESP_OK) {
        if (info.state == TWAI_STATE_STOPPED) {
            twai_start();
        } else if (info.state == TWAI_STATE_BUS_OFF) {
            static uint32_t last_recovery_ms;
            uint32_t now = now_ms();
            if (now - last_recovery_ms > 2000) {
                last_recovery_ms = now;
                twai_initiate_recovery();
            }
        }
    }

    twai_message_t msg = {0};
    msg.identifier = frame->id & CANARD_CAN_EXT_ID_MASK;
    msg.extd = (frame->id & CANARD_CAN_FRAME_EFF) ? 1 : 0;
    msg.rtr = (frame->id & CANARD_CAN_FRAME_RTR) ? 1 : 0;
    msg.data_length_code = frame->data_len;
    memcpy(msg.data, frame->data, frame->data_len);

    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

static bool can_receive_frame(CanardCANFrame *frame) {
    if (!frame) {
        return false;
    }
    twai_message_t msg = {0};
    if (twai_receive(&msg, 0) != ESP_OK) {
        return false;
    }
    if (!msg.extd || msg.data_length_code > 8) {
        return false;
    }

    frame->id = (msg.identifier & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
    if (msg.rtr) {
        frame->id |= CANARD_CAN_FRAME_RTR;
    }
    frame->data_len = msg.data_length_code;
    frame->iface_id = 0;
    memcpy(frame->data, msg.data, frame->data_len);
    return true;
}

static void process_tx(void) {
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&s_canard)) != NULL;) {
        if (can_send_frame(txf)) {
            canardPopTxQueue(&s_canard);
            s_tx_fail_count = 0;
        } else {
            if (s_tx_fail_count < 8) {
                s_tx_fail_count++;
            } else {
                canardPopTxQueue(&s_canard);
            }
            break;
        }
    }
}

static void process_rx(void) {
    uint8_t count = 60;
    while (count--) {
        CanardCANFrame frame = {0};
        if (!can_receive_frame(&frame)) {
            break;
        }
        (void)canardHandleRxFrame(&s_canard, &frame, now_us());
    }
}

static void node_status_send(void) {
    struct uavcan_protocol_NodeStatus pkt = {
        .uptime_sec = now_ms() / 1000U,
        .health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        .mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
        .sub_mode = 0,
        .vendor_specific_status_code = 0,
    };
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE] = {0};
    uint16_t len = (uint16_t)uavcan_protocol_NodeStatus_encode(&pkt, buffer);
    (void)canardBroadcast(&s_canard,
                          UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                          UAVCAN_PROTOCOL_NODESTATUS_ID,
                          &s_node_status_tid,
                          CANARD_TRANSFER_PRIORITY_LOW,
                          buffer,
                          len);
}

static void arm_status_send(void) {
    char reason[50] = {0};
    bool ok = rid_state_arm_status(reason, sizeof(reason));

    struct dronecan_remoteid_ArmStatus pkt = {
        .status = ok ? DRONECAN_REMOTEID_ARMSTATUS_ODID_ARM_STATUS_GOOD_TO_ARM
                     : DRONECAN_REMOTEID_ARMSTATUS_ODID_ARM_STATUS_FAIL_GENERIC,
    };
    size_t len_reason = strnlen(reason, sizeof(reason));
    pkt.error.len = (uint8_t)(len_reason > sizeof(pkt.error.data) ? sizeof(pkt.error.data) : len_reason);
    memcpy(pkt.error.data, reason, pkt.error.len);

    uint8_t buffer[DRONECAN_REMOTEID_ARMSTATUS_MAX_SIZE] = {0};
    uint16_t len = (uint16_t)dronecan_remoteid_ArmStatus_encode(&pkt, buffer);
    (void)canardBroadcast(&s_canard,
                          DRONECAN_REMOTEID_ARMSTATUS_SIGNATURE,
                          DRONECAN_REMOTEID_ARMSTATUS_ID,
                          &s_arm_status_tid,
                          CANARD_TRANSFER_PRIORITY_LOW,
                          buffer,
                          len);
}

static void handle_get_node_info(CanardRxTransfer *transfer) {
    struct uavcan_protocol_GetNodeInfoResponse pkt = {0};
    const esp_app_desc_t *desc = esp_app_get_description();
    uint8_t uid[16] = {0};
    read_unique_id(uid);

    pkt.status.uptime_sec = now_ms() / 1000U;
    pkt.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    pkt.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    pkt.hardware_version.major = SERY_RID_CAN_BOARD_ID >> 8;
    pkt.hardware_version.minor = SERY_RID_CAN_BOARD_ID & 0xFF;
    memcpy(pkt.hardware_version.unique_id, uid, sizeof(pkt.hardware_version.unique_id));
    pkt.software_version.major = 1;
    pkt.software_version.minor = 0;
    if (desc && desc->version[0] >= '0' && desc->version[0] <= '9') {
        pkt.software_version.major = (uint8_t)(desc->version[0] - '0');
    }

    const char *name = SERY_RID_CAN_NODE_NAME;
    size_t name_len = strlen(name);
    if (name_len > sizeof(pkt.name.data)) {
        name_len = sizeof(pkt.name.data);
    }
    pkt.name.len = (uint8_t)name_len;
    memcpy(pkt.name.data, name, name_len);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE] = {0};
    uint16_t len = (uint16_t)uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);
    (void)canardRequestOrRespond(&s_canard,
                                 transfer->source_node_id,
                                 UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                 UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                 &transfer->transfer_id,
                                 transfer->priority,
                                 CanardResponse,
                                 buffer,
                                 len);
}

static void handle_restart_node(CanardRxTransfer *transfer) {
    struct uavcan_protocol_RestartNodeResponse pkt = {
        .ok = true,
    };
    uint8_t buffer[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE] = {0};
    uint16_t len = (uint16_t)uavcan_protocol_RestartNodeResponse_encode(&pkt, buffer);
    (void)canardRequestOrRespond(&s_canard,
                                 transfer->source_node_id,
                                 UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE,
                                 UAVCAN_PROTOCOL_RESTARTNODE_ID,
                                 &transfer->transfer_id,
                                 transfer->priority,
                                 CanardResponse,
                                 buffer,
                                 len);
    process_tx();
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_restart();
}

static void handle_basic_id(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_BasicID pkt = {0};
    if (dronecan_remoteid_BasicID_decode(transfer, &pkt) ||
        pkt.uas_id.len == 0 ||
        pkt.id_type == 0 ||
        pkt.id_type > MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID) {
        return;
    }

    mavlink_open_drone_id_basic_id_t msg = {0};
    copy_bytes(msg.id_or_mac, sizeof(msg.id_or_mac), pkt.id_or_mac.data, pkt.id_or_mac.len);
    msg.id_type = pkt.id_type;
    msg.ua_type = pkt.ua_type;
    copy_bytes(msg.uas_id, sizeof(msg.uas_id), pkt.uas_id.data, pkt.uas_id.len);
    rid_state_update_basic_id(&msg);
}

static void handle_location(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_Location pkt = {0};
    if (dronecan_remoteid_Location_decode(transfer, &pkt)) {
        return;
    }

    mavlink_open_drone_id_location_t msg = {0};
    copy_bytes(msg.id_or_mac, sizeof(msg.id_or_mac), pkt.id_or_mac.data, pkt.id_or_mac.len);
    msg.status = pkt.status;
    msg.direction = pkt.direction;
    msg.speed_horizontal = pkt.speed_horizontal;
    msg.speed_vertical = pkt.speed_vertical;
    msg.latitude = pkt.latitude;
    msg.longitude = pkt.longitude;
    msg.altitude_barometric = pkt.altitude_barometric;
    msg.altitude_geodetic = pkt.altitude_geodetic;
    msg.height_reference = pkt.height_reference;
    msg.height = pkt.height;
    msg.horizontal_accuracy = pkt.horizontal_accuracy;
    msg.vertical_accuracy = pkt.vertical_accuracy;
    msg.barometer_accuracy = pkt.barometer_accuracy;
    msg.speed_accuracy = pkt.speed_accuracy;
    msg.timestamp = pkt.timestamp;
    msg.timestamp_accuracy = pkt.timestamp_accuracy;
    rid_state_update_location(&msg);
}

static void handle_self_id(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_SelfID pkt = {0};
    if (dronecan_remoteid_SelfID_decode(transfer, &pkt)) {
        return;
    }

    mavlink_open_drone_id_self_id_t msg = {0};
    copy_bytes(msg.id_or_mac, sizeof(msg.id_or_mac), pkt.id_or_mac.data, pkt.id_or_mac.len);
    msg.description_type = pkt.description_type;
    copy_bytes((uint8_t *)msg.description,
               sizeof(msg.description),
               pkt.description.data,
               pkt.description.len);
    rid_state_update_self_id(&msg);
}

static void handle_system(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_System pkt = {0};
    if (dronecan_remoteid_System_decode(transfer, &pkt)) {
        return;
    }

    mavlink_open_drone_id_system_t msg = {0};
    copy_bytes(msg.id_or_mac, sizeof(msg.id_or_mac), pkt.id_or_mac.data, pkt.id_or_mac.len);
    msg.operator_location_type = pkt.operator_location_type;
    msg.classification_type = pkt.classification_type;
    msg.operator_latitude = pkt.operator_latitude;
    msg.operator_longitude = pkt.operator_longitude;
    msg.area_count = pkt.area_count;
    msg.area_radius = pkt.area_radius;
    msg.area_ceiling = pkt.area_ceiling;
    msg.area_floor = pkt.area_floor;
    msg.category_eu = pkt.category_eu;
    msg.class_eu = pkt.class_eu;
    msg.operator_altitude_geo = pkt.operator_altitude_geo;
    msg.timestamp = pkt.timestamp;
    rid_state_update_system(&msg);
}

static void handle_operator_id(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_OperatorID pkt = {0};
    if (dronecan_remoteid_OperatorID_decode(transfer, &pkt)) {
        return;
    }

    mavlink_open_drone_id_operator_id_t msg = {0};
    copy_bytes(msg.id_or_mac, sizeof(msg.id_or_mac), pkt.id_or_mac.data, pkt.id_or_mac.len);
    msg.operator_id_type = pkt.operator_id_type;
    copy_bytes((uint8_t *)msg.operator_id,
               sizeof(msg.operator_id),
               pkt.operator_id.data,
               pkt.operator_id.len);
    rid_state_update_operator_id(&msg);
}

static bool param_set_from_value(const rid_param_t *param,
                                 const struct uavcan_protocol_param_Value *value) {
    if (!param || !value || value->union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        return false;
    }

    switch (param->info->type) {
    case RID_PARAM_UINT8:
    case RID_PARAM_INT8:
    case RID_PARAM_UINT32:
        if (value->union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            return false;
        }
        return cfg_param_set_as_float(param, (float)value->integer_value);

    case RID_PARAM_FLOAT:
        if (value->union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
            return false;
        }
        return cfg_param_set_as_float(param, value->real_value);

    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64: {
        if (value->union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE) {
            return false;
        }
        char str[129] = {0};
        size_t len = value->string_value.len;
        if (len >= sizeof(str)) {
            len = sizeof(str) - 1;
        }
        memcpy(str, value->string_value.data, len);
        return cfg_param_set_by_name_string(param->info->name, str);
    }

    default:
        return false;
    }
}

static void fill_param_value(struct uavcan_protocol_param_Value *dst,
                             const rid_param_t *param,
                             bool default_value) {
    memset(dst, 0, sizeof(*dst));
    if (!param) {
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        return;
    }

    switch (param->info->type) {
    case RID_PARAM_UINT8:
    case RID_PARAM_INT8:
    case RID_PARAM_UINT32:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        if (default_value) {
            dst->integer_value = (int64_t)param->info->default_value;
        } else {
            float v = 0.0f;
            cfg_param_get_as_float(param, &v);
            dst->integer_value = (int64_t)v;
        }
        break;

    case RID_PARAM_FLOAT:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
        dst->real_value = default_value ? param->info->default_value : 0.0f;
        if (!default_value) {
            cfg_param_get_as_float(param, &dst->real_value);
        }
        break;

    case RID_PARAM_CHAR20:
    case RID_PARAM_CHAR64: {
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
        const char *s = default_value ? "" : cfg_param_get_string(param);
        if (!s) {
            s = "";
        }
        size_t len = strlen(s);
        if (len > sizeof(dst->string_value.data)) {
            len = sizeof(dst->string_value.data);
        }
        dst->string_value.len = (uint8_t)len;
        memcpy(dst->string_value.data, s, len);
        break;
    }

    default:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        break;
    }
}

static void fill_numeric_value(struct uavcan_protocol_param_NumericValue *dst,
                               const rid_param_t *param,
                               bool max_value) {
    memset(dst, 0, sizeof(*dst));
    if (!param) {
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        return;
    }

    float value = max_value ? param->info->max_value : param->info->min_value;
    switch (param->info->type) {
    case RID_PARAM_UINT8:
    case RID_PARAM_INT8:
    case RID_PARAM_UINT32:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
        dst->integer_value = (int64_t)value;
        break;

    case RID_PARAM_FLOAT:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
        dst->real_value = value;
        break;

    default:
        dst->union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        break;
    }
}

static void handle_param_getset(CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_GetSetRequest req = {0};
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    const rid_param_t *param = NULL;
    if (req.name.len > 0) {
        char name[17] = {0};
        size_t len = req.name.len;
        if (len > sizeof(name) - 1) {
            len = sizeof(name) - 1;
        }
        memcpy(name, req.name.data, len);
        param = cfg_param_find(name);
    } else {
        param = cfg_param_find_by_index(req.index);
    }

    if (param && (param->info->flags & RID_PARAM_FLAG_HIDDEN)) {
        param = NULL;
    }

    if (param && req.name.len > 0 &&
        req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        if (cfg_get()->lock_level > 0) {
            ESP_LOGW(TAG, "parameters locked");
        } else if (!param_set_from_value(param, &req.value)) {
            ESP_LOGW(TAG, "failed to set parameter %s", param->info->name);
        }
    }

    struct uavcan_protocol_param_GetSetResponse res = {0};
    if (param) {
        fill_param_value(&res.value, param, false);
        fill_param_value(&res.default_value, param, true);
        fill_numeric_value(&res.max_value, param, true);
        fill_numeric_value(&res.min_value, param, false);
        size_t name_len = strlen(param->info->name);
        res.name.len = (uint8_t)name_len;
        memcpy(res.name.data, param->info->name, name_len);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE] = {0};
    uint16_t len = (uint16_t)uavcan_protocol_param_GetSetResponse_encode(&res, buffer);
    (void)canardRequestOrRespond(&s_canard,
                                 transfer->source_node_id,
                                 UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                                 UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                                 &transfer->transfer_id,
                                 transfer->priority,
                                 CanardResponse,
                                 buffer,
                                 len);
}

static void append_config_kv(uint8_t *data, uint8_t *len, const rid_param_t *param) {
    if (!data || !len || !param || (param->info->flags & RID_PARAM_FLAG_HIDDEN)) {
        return;
    }

    char line[96] = {0};
    const char *s = cfg_param_get_string(param);
    if (s) {
        snprintf(line, sizeof(line), "%s=%s", param->info->name, s);
    } else {
        float value = 0.0f;
        if (!cfg_param_get_as_float(param, &value)) {
            return;
        }
        snprintf(line, sizeof(line), "%s=%.6g", param->info->name, (double)value);
    }

    size_t n = strlen(line) + 1;
    if ((size_t)*len + n > 220) {
        return;
    }
    memcpy(&data[*len], line, n);
    *len += (uint8_t)n;
}

static void handle_secure_command(CanardRxTransfer *transfer) {
    struct dronecan_remoteid_SecureCommandRequest req = {0};
    if (dronecan_remoteid_SecureCommandRequest_decode(transfer, &req)) {
        return;
    }

    struct dronecan_remoteid_SecureCommandResponse res = {
        .sequence = req.sequence,
        .operation = req.operation,
        .result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_UNSUPPORTED,
    };

    if (req.sig_length > req.data.len) {
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_DENIED;
        goto send_reply;
    }

    uint8_t data_len = (uint8_t)(req.data.len - req.sig_length);
    if (!rid_secure_check_signature(req.sig_length,
                                    data_len,
                                    req.sequence,
                                    req.operation,
                                    req.data.data)) {
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_DENIED;
        goto send_reply;
    }

    switch (req.operation) {
    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_SESSION_KEY:
    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_REMOTEID_SESSION_KEY:
        rid_secure_make_session_key(res.data.data);
        res.data.len = 8;
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED;
        break;

    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_PUBLIC_KEYS: {
        if (data_len != 2 || cfg_no_public_keys()) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        uint8_t key_idx = req.data.data[0];
        uint8_t num_keys = req.data.data[1];
        const uint8_t max_fetch = (sizeof(res.data.data) - 1) / 32;
        if (num_keys == 0 || num_keys > max_fetch || key_idx >= 5 || key_idx + num_keys > 5) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        res.data.data[0] = key_idx;
        uint8_t copied = 0;
        for (uint8_t i = 0; i < num_keys; i++) {
            if (cfg_get_public_key((uint8_t)(key_idx + i), &res.data.data[1 + copied * 32])) {
                copied++;
            }
        }
        res.data.len = (uint8_t)(1 + copied * 32);
        res.result = copied > 0 ? DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED
                                : DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
        break;
    }

    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_SET_PUBLIC_KEYS: {
        if (data_len < 33) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        uint8_t key_idx = req.data.data[0];
        uint8_t num_keys = (data_len - 1) / 32;
        if (num_keys == 0 || key_idx >= 5 || key_idx + num_keys > 5) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        bool failed = false;
        for (uint8_t i = 0; i < num_keys; i++) {
            failed |= !cfg_set_public_key((uint8_t)(key_idx + i), &req.data.data[1 + i * 32]);
        }
        res.result = failed ? DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED
                            : DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED;
        break;
    }

    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_REMOVE_PUBLIC_KEYS: {
        if (data_len != 2) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        uint8_t key_idx = req.data.data[0];
        uint8_t num_keys = req.data.data[1];
        if (num_keys == 0 || key_idx >= 5 || key_idx + num_keys > 5) {
            res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
            break;
        }
        for (uint8_t i = 0; i < num_keys; i++) {
            cfg_remove_public_key((uint8_t)(key_idx + i));
        }
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED;
        break;
    }

    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_GET_REMOTEID_CONFIG:
        for (uint16_t i = 0;; i++) {
            const rid_param_t *param = cfg_param_find_by_index(i);
            if (!param) {
                break;
            }
            append_config_kv(res.data.data, &res.data.len, param);
        }
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED;
        break;

    case DRONECAN_REMOTEID_SECURECOMMAND_REQUEST_SECURE_COMMAND_SET_REMOTEID_CONFIG: {
        char command_buf[221] = {0};
        memcpy(command_buf, req.data.data, data_len);
        int16_t remaining = data_len;
        char *command = command_buf;
        res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_ACCEPTED;
        while (remaining > 0 && *command != 0) {
            size_t cmd_len = strlen(command);
            char *eq = strchr(command, '=');
            if (eq) {
                *eq = 0;
                if (!cfg_param_set_by_name_string(command, eq + 1)) {
                    res.result = DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_RESULT_FAILED;
                }
            }
            command += cmd_len + 1;
            remaining -= (int16_t)(cmd_len + 1);
        }
        break;
    }

    default:
        break;
    }

send_reply:
    {
        uint8_t buffer[DRONECAN_REMOTEID_SECURECOMMAND_RESPONSE_MAX_SIZE] = {0};
        uint16_t len = (uint16_t)dronecan_remoteid_SecureCommandResponse_encode(&res, buffer);
        (void)canardRequestOrRespond(&s_canard,
                                     transfer->source_node_id,
                                     DRONECAN_REMOTEID_SECURECOMMAND_SIGNATURE,
                                     DRONECAN_REMOTEID_SECURECOMMAND_ID,
                                     &transfer->transfer_id,
                                     transfer->priority,
                                     CanardResponse,
                                     buffer,
                                     len);
    }
}

static void handle_allocation_response(CanardRxTransfer *transfer) {
    s_next_node_id_allocation_request_ms =
        now_ms() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (esp_random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        s_node_id_allocation_unique_id_offset = 0;
        return;
    }

    struct uavcan_protocol_dynamic_node_id_Allocation msg = {0};
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
        return;
    }

    uint8_t unique_id[16] = {0};
    read_unique_id(unique_id);
    if (memcmp(msg.unique_id.data, unique_id, msg.unique_id.len) != 0) {
        s_node_id_allocation_unique_id_offset = 0;
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        s_node_id_allocation_unique_id_offset = msg.unique_id.len;
        s_next_node_id_allocation_request_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
    } else if (msg.node_id > 0 && msg.node_id <= CANARD_MAX_NODE_ID) {
        canardSetLocalNodeID(&s_canard, msg.node_id);
        ESP_LOGI(TAG, "DroneCAN node ID allocated: %u", (unsigned)msg.node_id);
    }
}

static bool do_dynamic_node_allocation(void) {
    if (canardGetLocalNodeID(&s_canard) != CANARD_BROADCAST_NODE_ID) {
        return true;
    }

    uint32_t now = now_ms();
    if (s_next_node_id_allocation_request_ms != 0 &&
        now < s_next_node_id_allocation_request_ms) {
        return false;
    }

    s_next_node_id_allocation_request_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (esp_random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1] = {0};
    allocation_request[0] = s_node_id_allocation_unique_id_offset == 0 ? 1 : 0;

    uint8_t unique_id[16] = {0};
    read_unique_id(unique_id);
    uint8_t uid_size = (uint8_t)(sizeof(unique_id) - s_node_id_allocation_unique_id_offset);
    if (uid_size > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
        uid_size = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
    }
    memcpy(&allocation_request[1],
           &unique_id[s_node_id_allocation_unique_id_offset],
           uid_size);

    (void)canardBroadcast(&s_canard,
                          UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                          UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                          &s_dna_tid,
                          CANARD_TRANSFER_PRIORITY_LOW,
                          allocation_request,
                          (uint16_t)(uid_size + 1));
    s_node_id_allocation_unique_id_offset = 0;
    return false;
}

static void on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer) {
    (void)ins;

    if (canardGetLocalNodeID(&s_canard) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(transfer);
        break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        handle_restart_node(transfer);
        break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        handle_param_getset(transfer);
        break;
    case DRONECAN_REMOTEID_SECURECOMMAND_ID:
        handle_secure_command(transfer);
        break;
    case DRONECAN_REMOTEID_BASICID_ID:
        handle_basic_id(transfer);
        break;
    case DRONECAN_REMOTEID_LOCATION_ID:
        handle_location(transfer);
        break;
    case DRONECAN_REMOTEID_SELFID_ID:
        handle_self_id(transfer);
        break;
    case DRONECAN_REMOTEID_SYSTEM_ID:
        handle_system(transfer);
        break;
    case DRONECAN_REMOTEID_OPERATORID_ID:
        handle_operator_id(transfer);
        break;
    default:
        break;
    }
}

static bool should_accept_transfer(const CanardInstance *ins,
                                   uint64_t *out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id) {
    (void)transfer_type;
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID &&
        data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
        *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
        return true;
    }

    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_SECURECOMMAND_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_SECURECOMMAND_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_BASICID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_BASICID_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_LOCATION_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_LOCATION_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_SELFID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_SELFID_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_SYSTEM_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_SYSTEM_SIGNATURE;
        return true;
    case DRONECAN_REMOTEID_OPERATORID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_OPERATORID_SIGNATURE;
        return true;
    default:
        return false;
    }
}

static void dronecan_task(void *arg) {
    (void)arg;
    TickType_t loop_delay = pdMS_TO_TICKS(5);
    if (loop_delay == 0) {
        loop_delay = 1;
    }

    while (true) {
        uint32_t now = now_ms();
        if (do_dynamic_node_allocation() && now - s_last_node_status_ms >= 1000) {
            s_last_node_status_ms = now;
            node_status_send();
            arm_status_send();
        }
        process_tx();
        process_rx();
        if (now - s_last_cleanup_ms >= 1000) {
            s_last_cleanup_ms = now;
            canardCleanupStaleTransfers(&s_canard, now_us());
        }
        vTaskDelay(loop_delay);
    }
}

esp_err_t rid_dronecan_start(void) {
    esp_err_t err = can_driver_start();
    if (err == ESP_ERR_NOT_SUPPORTED) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "DroneCAN TWAI start failed");

    canardInit(&s_canard,
               s_canard_pool,
               sizeof(s_canard_pool),
               on_transfer_received,
               should_accept_transfer,
               NULL);

    const rid_config_t *cfg = cfg_get();
    if (cfg->can_node > 0 && cfg->can_node <= CANARD_MAX_NODE_ID) {
        canardSetLocalNodeID(&s_canard, cfg->can_node);
    }

    xTaskCreateStatic(dronecan_task,
                      "rid_dronecan",
                      SERY_RID_TASK_STACK_WORDS,
                      NULL,
                      5,
                      s_dronecan_stack,
                      &s_dronecan_tcb);
    return ESP_OK;
}
