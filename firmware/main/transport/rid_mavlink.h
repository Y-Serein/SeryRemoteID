#ifndef SERY_RID_MAVLINK_H
#define SERY_RID_MAVLINK_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rid_mavlink_start(void);
uint8_t rid_mavlink_system_id(void);

#ifdef __cplusplus
}
#endif

#endif
