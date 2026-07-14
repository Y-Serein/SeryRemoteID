#include "transport/rid_mavlink.h"

#include <stdio.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "driver/uart.h"
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "protocol/rid_state.h"
#include "security/rid_secure.h"

static const char *TAG = "rid_mavlink";

#define ARRAY_LEN(_a) (sizeof(_a) / sizeof((_a)[0]))
#define SERY_RID_USB_SERIAL_BUFFER_BYTES 2048

typedef enum {
    RID_MAVLINK_LINK_UART,
    RID_MAVLINK_LINK_USB_SERIAL_JTAG,
} rid_mavlink_link_type_t;

typedef struct {
    const char *name;
    rid_mavlink_link_type_t type;
    mavlink_channel_t channel;
    bool enabled;
    mavlink_status_t parse_status;
    uint16_t param_stream_index;
    uint32_t param_stream_last_ms;
} rid_mavlink_link_t;

static StackType_t s_mavlink_stack[SERY_RID_TASK_STACK_WORDS];
static StaticTask_t s_mavlink_tcb;
static uint8_t s_system_id;
static rid_mavlink_diagnostics_t s_diagnostics;

static rid_mavlink_link_t s_links[] = {
    {
        .name = "UART",
        .type = RID_MAVLINK_LINK_UART,
        .channel = MAVLINK_COMM_0,
        .enabled = true,
    },
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    {
        .name = "USB Serial/JTAG",
        .type = RID_MAVLINK_LINK_USB_SERIAL_JTAG,
        .channel = MAVLINK_COMM_1,
        .enabled = false,
    },
#endif
};

uint8_t rid_mavlink_system_id(void) {
    return s_system_id;
}

void rid_mavlink_get_diagnostics(rid_mavlink_diagnostics_t *out) {
    if (out) {
        *out = s_diagnostics;
    }
}

static void send_mavlink_message(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    if (!link || !link->enabled) {
        return;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    if (len == 0) {
        return;
    }

    switch (link->type) {
    case RID_MAVLINK_LINK_UART:
        uart_write_bytes(SERY_RID_UART_PORT, buffer, len);
        break;

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    case RID_MAVLINK_LINK_USB_SERIAL_JTAG: {
        size_t offset = 0;
        while (offset < len) {
            int written = usb_serial_jtag_write_bytes((const char *)&buffer[offset],
                                                      len - offset,
                                                      pdMS_TO_TICKS(20));
            if (written <= 0) {
                break;
            }
            offset += (size_t)written;
        }
        break;
    }
#endif

    default:
        break;
    }
}

static void send_heartbeat(rid_mavlink_link_t *link) {
    if (s_system_id == 0) {
        return;
    }
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(s_system_id,
                                    SERY_RID_MAVLINK_COMPID,
                                    (uint8_t)link->channel,
                                    &msg,
                                    MAV_TYPE_ODID,
                                    MAV_AUTOPILOT_INVALID,
                                    0,
                                    0,
                                    0);
    send_mavlink_message(link, &msg);
}

static void send_arm_status(rid_mavlink_link_t *link) {
    if (s_system_id == 0) {
        return;
    }
    char reason[50] = {0};
    bool ok = rid_state_arm_status(reason, sizeof(reason));
    mavlink_message_t msg;
    mavlink_msg_open_drone_id_arm_status_pack_chan(
        s_system_id,
        SERY_RID_MAVLINK_COMPID,
        (uint8_t)link->channel,
        &msg,
        ok ? MAV_ODID_ARM_STATUS_GOOD_TO_ARM : MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC,
        ok ? "" : reason);
    send_mavlink_message(link, &msg);
}

static void send_statustext(rid_mavlink_link_t *link, uint8_t severity, const char *text) {
    if (s_system_id == 0 || !text) {
        return;
    }
    mavlink_message_t msg;
    mavlink_msg_statustext_pack_chan(s_system_id,
                                     SERY_RID_MAVLINK_COMPID,
                                     (uint8_t)link->channel,
                                     &msg,
                                     severity,
                                     text,
                                     0,
                                     0);
    send_mavlink_message(link, &msg);
}

static void send_param_value(rid_mavlink_link_t *link, const rid_param_t *param) {
    if (s_system_id == 0 || !param) {
        return;
    }

    float value = 0.0f;
    if (!cfg_param_get_as_float(param, &value)) {
        return;
    }

    int16_t index = cfg_param_index_float(param);
    if (index < 0) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_param_value_pack_chan(s_system_id,
                                      SERY_RID_MAVLINK_COMPID,
                                      (uint8_t)link->channel,
                                      &msg,
                                      param->info->name,
                                      value,
                                      MAV_PARAM_TYPE_REAL32,
                                      cfg_param_count_float(),
                                      (uint16_t)index);
    send_mavlink_message(link, &msg);
}

static void learn_system_id_from_heartbeat(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    if (s_system_id != 0 || cfg_get()->mavlink_sysid != 0) {
        return;
    }
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);
    if (msg->sysid > 0 && heartbeat.type != MAV_TYPE_GCS) {
        s_system_id = msg->sysid;
        ESP_LOGI(TAG, "learned MAVLink system id %u from %s",
                 (unsigned)s_system_id,
                 link ? link->name : "unknown");
    }
}

static bool packet_targets_this_component(uint8_t target_system, uint8_t target_component) {
    if (s_system_id != 0 && target_system != 0 && target_system != s_system_id) {
        return false;
    }
    if (target_component != 0 && target_component != SERY_RID_MAVLINK_COMPID) {
        return false;
    }
    return true;
}

static void handle_param_request_read(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    mavlink_param_request_read_t pkt;
    mavlink_msg_param_request_read_decode(msg, &pkt);
    if (!packet_targets_this_component(pkt.target_system, pkt.target_component)) {
        return;
    }

    const rid_param_t *param = NULL;
    if (pkt.param_index < 0) {
        char name[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN + 1] = {0};
        memcpy(name, pkt.param_id, MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN);
        param = cfg_param_find(name);
    } else {
        param = cfg_param_find_float_by_index((uint16_t)pkt.param_index);
    }

    if (!param || (param->info->flags & RID_PARAM_FLAG_HIDDEN)) {
        return;
    }
    send_param_value(link, param);
}

static void handle_param_set(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    mavlink_param_set_t pkt;
    mavlink_msg_param_set_decode(msg, &pkt);
    if (!packet_targets_this_component(pkt.target_system, pkt.target_component)) {
        return;
    }
    if (pkt.param_type != MAV_PARAM_TYPE_REAL32) {
        return;
    }

    char name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1] = {0};
    memcpy(name, pkt.param_id, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);
    const rid_param_t *param = cfg_param_find(name);
    float current = 0.0f;
    if (!param || !cfg_param_get_as_float(param, &current)) {
        return;
    }

    const rid_config_t *cfg = cfg_get();
    if (cfg->lock_level > 0 &&
        (strcmp(param->info->name, "LOCK_LEVEL") != 0 ||
         (uint8_t)pkt.param_value <= (uint8_t)current)) {
        send_statustext(link, MAV_SEVERITY_ERROR, "Parameters locked");
    } else {
        if (!cfg_param_set_as_float(param, pkt.param_value)) {
            send_statustext(link, MAV_SEVERITY_ERROR, "Parameter set failed");
        }
    }
    send_param_value(link, param);
}

static void handle_param_request_list(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    mavlink_param_request_list_t pkt;
    mavlink_msg_param_request_list_decode(msg, &pkt);
    if (!packet_targets_this_component(pkt.target_system, pkt.target_component)) {
        return;
    }
    link->param_stream_index = 0;
    link->param_stream_last_ms = 1;
}

static void append_config_kv(uint8_t *data, uint8_t *len, const rid_param_t *param) {
    if (!param || !len || !data || (param->info->flags & RID_PARAM_FLAG_HIDDEN)) {
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
    if ((size_t)*len + n > MAVLINK_MSG_SECURE_COMMAND_REPLY_FIELD_DATA_LEN) {
        return;
    }
    memcpy(&data[*len], line, n);
    *len += (uint8_t)n;
}

static void handle_secure_command(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    mavlink_secure_command_t pkt;
    mavlink_msg_secure_command_decode(msg, &pkt);
    if (!packet_targets_this_component(pkt.target_system, pkt.target_component)) {
        return;
    }

    mavlink_secure_command_reply_t reply = {
        .sequence = pkt.sequence,
        .operation = pkt.operation,
        .result = MAV_RESULT_UNSUPPORTED,
        .data_length = 0,
    };

    if ((uint16_t)pkt.data_length + (uint16_t)pkt.sig_length > sizeof(pkt.data)) {
        reply.result = MAV_RESULT_DENIED;
        goto send_reply;
    }

    if (!rid_secure_check_signature(pkt.sig_length,
                                    pkt.data_length,
                                    pkt.sequence,
                                    pkt.operation,
                                    pkt.data)) {
        reply.result = MAV_RESULT_DENIED;
        goto send_reply;
    }

    switch (pkt.operation) {
    case SECURE_COMMAND_GET_SESSION_KEY:
    case SECURE_COMMAND_GET_REMOTEID_SESSION_KEY:
        rid_secure_make_session_key(reply.data);
        reply.data_length = 8;
        reply.result = MAV_RESULT_ACCEPTED;
        break;

    case SECURE_COMMAND_GET_PUBLIC_KEYS: {
        if (pkt.data_length != 2) {
            reply.result = MAV_RESULT_UNSUPPORTED;
            break;
        }
        uint8_t key_idx = pkt.data[0];
        uint8_t num_keys = pkt.data[1];
        uint8_t max_fetch = (MAVLINK_MSG_SECURE_COMMAND_REPLY_FIELD_DATA_LEN - 1) / 32;
        if (key_idx >= 5 || num_keys == 0 || num_keys > max_fetch || key_idx + num_keys > 5) {
            reply.result = MAV_RESULT_FAILED;
            break;
        }
        reply.data[0] = key_idx;
        uint8_t copied = 0;
        for (uint8_t i = 0; i < num_keys; i++) {
            if (cfg_get_public_key((uint8_t)(key_idx + i), &reply.data[1 + copied * 32])) {
                copied++;
            }
        }
        reply.data_length = (uint8_t)(1 + copied * 32);
        reply.result = copied > 0 ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        break;
    }

    case SECURE_COMMAND_SET_PUBLIC_KEYS: {
        if (pkt.data_length < 33) {
            reply.result = MAV_RESULT_FAILED;
            break;
        }
        uint8_t key_idx = pkt.data[0];
        uint8_t num_keys = (pkt.data_length - 1) / 32;
        if (num_keys == 0 || key_idx >= 5 || key_idx + num_keys > 5) {
            reply.result = MAV_RESULT_FAILED;
            break;
        }
        bool failed = false;
        for (uint8_t i = 0; i < num_keys; i++) {
            failed |= !cfg_set_public_key((uint8_t)(key_idx + i), &pkt.data[1 + i * 32]);
        }
        reply.result = failed ? MAV_RESULT_FAILED : MAV_RESULT_ACCEPTED;
        break;
    }

    case SECURE_COMMAND_REMOVE_PUBLIC_KEYS: {
        if (pkt.data_length != 2) {
            reply.result = MAV_RESULT_FAILED;
            break;
        }
        uint8_t key_idx = pkt.data[0];
        uint8_t num_keys = pkt.data[1];
        if (num_keys == 0 || key_idx >= 5 || key_idx + num_keys > 5) {
            reply.result = MAV_RESULT_FAILED;
            break;
        }
        for (uint8_t i = 0; i < num_keys; i++) {
            cfg_remove_public_key((uint8_t)(key_idx + i));
        }
        reply.result = MAV_RESULT_ACCEPTED;
        break;
    }

    case SECURE_COMMAND_SET_REMOTEID_CONFIG: {
        int16_t data_len = pkt.data_length;
        char command_buf[MAVLINK_MSG_SECURE_COMMAND_FIELD_DATA_LEN + 1] = {0};
        memcpy(command_buf, pkt.data, pkt.data_length);
        reply.result = MAV_RESULT_ACCEPTED;
        char *command = command_buf;
        while (data_len > 0 && *command != 0) {
            size_t cmd_len = strlen(command);
            char *eq = strchr(command, '=');
            if (eq) {
                *eq = 0;
                if (!cfg_param_set_by_name_string(command, eq + 1)) {
                    reply.result = MAV_RESULT_FAILED;
                }
            }
            command += cmd_len + 1;
            data_len -= (int16_t)(cmd_len + 1);
        }
        break;
    }

    case SECURE_COMMAND_GET_REMOTEID_CONFIG:
        for (uint16_t i = 0;; i++) {
            const rid_param_t *param = cfg_param_find_by_index(i);
            if (!param) {
                break;
            }
            append_config_kv(reply.data, &reply.data_length, param);
        }
        reply.result = MAV_RESULT_ACCEPTED;
        break;

    default:
        break;
    }

send_reply:
    if (s_system_id != 0) {
        mavlink_message_t reply_msg;
        mavlink_msg_secure_command_reply_encode_chan(s_system_id,
                                                     SERY_RID_MAVLINK_COMPID,
                                                     (uint8_t)link->channel,
                                                     &reply_msg,
                                                     &reply);
        send_mavlink_message(link, &reply_msg);
    }
}

static void process_packet(rid_mavlink_link_t *link, const mavlink_message_t *msg) {
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        learn_system_id_from_heartbeat(link, msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t pkt;
        mavlink_msg_open_drone_id_basic_id_decode(msg, &pkt);
        rid_state_update_basic_id(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: {
        mavlink_open_drone_id_location_t pkt;
        mavlink_msg_open_drone_id_location_decode(msg, &pkt);
        rid_state_update_location(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION: {
        mavlink_open_drone_id_authentication_t pkt;
        mavlink_msg_open_drone_id_authentication_decode(msg, &pkt);
        rid_state_update_authentication(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: {
        mavlink_open_drone_id_self_id_t pkt;
        mavlink_msg_open_drone_id_self_id_decode(msg, &pkt);
        rid_state_update_self_id(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: {
        mavlink_open_drone_id_system_t pkt;
        mavlink_msg_open_drone_id_system_decode(msg, &pkt);
        rid_state_update_system(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: {
        mavlink_open_drone_id_system_update_t pkt;
        mavlink_msg_open_drone_id_system_update_decode(msg, &pkt);
        rid_state_update_system_update(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: {
        mavlink_open_drone_id_operator_id_t pkt;
        mavlink_msg_open_drone_id_operator_id_decode(msg, &pkt);
        rid_state_update_operator_id(&pkt);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        handle_param_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(link, msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(link, msg);
        break;
    case MAVLINK_MSG_ID_SECURE_COMMAND:
        handle_secure_command(link, msg);
        break;
    default:
        break;
    }
}

static int read_link(rid_mavlink_link_t *link, uint8_t *rx, size_t rx_len) {
    if (!link || !link->enabled) {
        return 0;
    }

    switch (link->type) {
    case RID_MAVLINK_LINK_UART:
        return uart_read_bytes(SERY_RID_UART_PORT, rx, rx_len, pdMS_TO_TICKS(20));

#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    case RID_MAVLINK_LINK_USB_SERIAL_JTAG:
        return usb_serial_jtag_read_bytes(rx, rx_len, 0);
#endif

    default:
        return 0;
    }
}

static void service_link_rx(rid_mavlink_link_t *link, uint8_t *rx, size_t rx_len) {
    mavlink_message_t msg;
    int n = read_link(link, rx, rx_len);
    if (n > 0 && link->type == RID_MAVLINK_LINK_UART) {
        s_diagnostics.rx_bytes += (uint32_t)n;
    }
    for (int i = 0; i < n; i++) {
        uint8_t parse_errors = link->parse_status.parse_error;
        if (mavlink_parse_char((uint8_t)link->channel, rx[i], &msg, &link->parse_status)) {
            if (link->type == RID_MAVLINK_LINK_UART) {
                s_diagnostics.valid_frames++;
                s_diagnostics.last_frame_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                s_diagnostics.last_message_id = msg.msgid;
                s_diagnostics.last_system_id = msg.sysid;
                s_diagnostics.last_component_id = msg.compid;
            }
            process_packet(link, &msg);
        }
        if (link->type == RID_MAVLINK_LINK_UART &&
            link->parse_status.parse_error != parse_errors) {
            s_diagnostics.parse_errors++;
        }
    }
}

static void send_periodic_status(uint32_t now, uint32_t *last_heartbeat_ms) {
    if (s_system_id == 0 || now - *last_heartbeat_ms < 1000) {
        return;
    }
    *last_heartbeat_ms = now;
    for (size_t i = 0; i < ARRAY_LEN(s_links); i++) {
        if (!s_links[i].enabled) {
            continue;
        }
        send_heartbeat(&s_links[i]);
        send_arm_status(&s_links[i]);
    }
}

static void service_param_streams(uint32_t now) {
    for (size_t i = 0; i < ARRAY_LEN(s_links); i++) {
        rid_mavlink_link_t *link = &s_links[i];
        if (!link->enabled || link->param_stream_last_ms == 0 ||
            now - link->param_stream_last_ms < 50) {
            continue;
        }

        const rid_param_t *param = cfg_param_find_float_by_index(link->param_stream_index);
        if (param) {
            link->param_stream_last_ms = now;
            send_param_value(link, param);
            link->param_stream_index++;
        } else {
            link->param_stream_last_ms = 0;
        }
    }
}

static void mavlink_task(void *arg) {
    (void)arg;
    uint8_t rx[256];
    uint32_t last_heartbeat_ms = 0;
    uint32_t last_wait_log_ms = 0;

    ESP_LOGI(TAG, "MAVLink task started");

    while (true) {
        for (size_t i = 0; i < ARRAY_LEN(s_links); i++) {
            service_link_rx(&s_links[i], rx, sizeof(rx));
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        send_periodic_status(now, &last_heartbeat_ms);
        if (s_system_id == 0 && now - last_wait_log_ms >= 2000) {
            last_wait_log_ms = now;
            ESP_LOGI(TAG, "waiting for vehicle heartbeat");
        }

        service_param_streams(now);
    }
}

static void configure_usb_serial_jtag_link(void) {
#if CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    rid_mavlink_link_t *link = NULL;
    for (size_t i = 0; i < ARRAY_LEN(s_links); i++) {
        if (s_links[i].type == RID_MAVLINK_LINK_USB_SERIAL_JTAG) {
            link = &s_links[i];
            break;
        }
    }
    if (!link) {
        return;
    }

    usb_serial_jtag_driver_config_t usb_config = {
        .rx_buffer_size = SERY_RID_USB_SERIAL_BUFFER_BYTES,
        .tx_buffer_size = SERY_RID_USB_SERIAL_BUFFER_BYTES,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&usb_config);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "USB Serial/JTAG driver already installed, reusing it for MAVLink");
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "USB Serial/JTAG MAVLink disabled: %s", esp_err_to_name(err));
        return;
    }

    link->enabled = true;
    ESP_LOGI(TAG, "MAVLink USB Serial/JTAG configured");
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    ESP_LOGW(TAG, "USB Serial/JTAG MAVLink shares secondary console output");
#endif
#else
    ESP_LOGI(TAG, "USB Serial/JTAG MAVLink not supported by this target");
#endif
}

esp_err_t rid_mavlink_start(void) {
    const rid_config_t *cfg = cfg_get();
    s_system_id = cfg->mavlink_sysid;
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));

    const uart_config_t uart_config = {
        .baud_rate = (int)cfg->uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(SERY_RID_UART_PORT,
                                            SERY_RID_UART_RX_BUFFER_BYTES,
                                            0,
                                            0,
                                            NULL,
                                            0),
                        TAG,
                        "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(SERY_RID_UART_PORT, &uart_config),
                        TAG,
                        "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(SERY_RID_UART_PORT,
                                     cfg->uart_tx_gpio,
                                     cfg->uart_rx_gpio,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG,
                        "uart_set_pin failed");

    configure_usb_serial_jtag_link();

    xTaskCreateStatic(mavlink_task,
                      "rid_mavlink",
                      SERY_RID_TASK_STACK_WORDS,
                      NULL,
                      5,
                      s_mavlink_stack,
                      &s_mavlink_tcb);

    ESP_LOGI(TAG, "MAVLink UART configured: baud=%lu tx=%d rx=%d sysid=%u",
             (unsigned long)cfg->uart_baud,
             cfg->uart_tx_gpio,
             cfg->uart_rx_gpio,
             (unsigned)s_system_id);
    return ESP_OK;
}
