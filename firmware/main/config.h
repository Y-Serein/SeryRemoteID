#ifndef SERY_RID_CONFIG_H
#define SERY_RID_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"

/* Default board profile: ESP32-S3 dev board, matching ArduRemoteID's S3 pins. */
#define SERY_RID_UART_PORT              UART_NUM_1
#define SERY_RID_UART_TX_GPIO           18
#define SERY_RID_UART_RX_GPIO           17
#define SERY_RID_UART_BAUD_DEFAULT      57600
#define SERY_RID_UART_RX_BUFFER_BYTES   2048

#define SERY_RID_WIFI_CHANNEL_DEFAULT   6
#define SERY_RID_WIFI_POWER_DBM_DEFAULT 20.0f
#define SERY_RID_WIFI_NAN_RATE_HZ       1.0f
#define SERY_RID_WIFI_BEACON_RATE_HZ    1.0f
#define SERY_RID_WIFI_SSID_PREFIX       "RID"
#define SERY_RID_WIFI_PASSWORD_DEFAULT  "ArduRemoteID"

#define SERY_RID_BT4_RATE_HZ            1.0f
#define SERY_RID_BT4_POWER_DBM_DEFAULT  18.0f
#define SERY_RID_BT5_RATE_HZ            1.0f
#define SERY_RID_BT5_POWER_DBM_DEFAULT  18.0f

#define SERY_RID_BROADCAST_POWERUP      false
#define SERY_RID_WEBSERVER_ENABLE_DEFAULT 1

#define SERY_RID_LOCK_LEVEL_DEFAULT     0
#define SERY_RID_CAN_NODE_DEFAULT       0
#define SERY_RID_CAN_TERMINATE_DEFAULT  0
#define SERY_RID_CAN_TX_GPIO            47
#define SERY_RID_CAN_RX_GPIO            38
#define SERY_RID_CAN_TERM_GPIO          (-1)
#define SERY_RID_CAN_TERM_ON_LEVEL      1
#define SERY_RID_CAN_BITRATE            1000000
#define SERY_RID_CAN_BOARD_ID           1
#define SERY_RID_CAN_NODE_NAME          "SeryRemoteID"

#define SERY_RID_STATUS_LED_GPIO        (-1)
#define SERY_RID_STATUS_LED_ON_LEVEL    1

#define SERY_RID_LOCATION_STALE_MS      5000u
#define SERY_RID_SYSTEM_STALE_MS        5000u
#define SERY_RID_AUX_STALE_MS           22000u

#define SERY_RID_MAVLINK_SYSID_DEFAULT  0
#define SERY_RID_MAVLINK_COMPID         236

#define SERY_RID_TASK_STACK_WORDS       4096

#endif
