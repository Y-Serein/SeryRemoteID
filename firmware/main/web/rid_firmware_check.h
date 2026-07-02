#ifndef SERY_RID_FIRMWARE_CHECK_H
#define SERY_RID_FIRMWARE_CHECK_H

#include <stdbool.h>

#include "esp_partition.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rid_firmware_check_ota_partition(const esp_partition_t *part);

#ifdef __cplusplus
}
#endif

#endif

