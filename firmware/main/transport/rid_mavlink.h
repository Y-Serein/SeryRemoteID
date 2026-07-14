#ifndef SERY_RID_MAVLINK_H
#define SERY_RID_MAVLINK_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t parse_errors;
    uint32_t last_frame_ms;
    uint32_t last_message_id;
    uint8_t last_system_id;
    uint8_t last_component_id;
} rid_mavlink_diagnostics_t;

esp_err_t rid_mavlink_start(void);
uint8_t rid_mavlink_system_id(void);
void rid_mavlink_get_diagnostics(rid_mavlink_diagnostics_t *out);

#ifdef __cplusplus
}
#endif

#endif
