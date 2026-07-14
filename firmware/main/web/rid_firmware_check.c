#include "web/rid_firmware_check.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "monocypher.h"

static const char *TAG = "rid_fw_check";

typedef struct {
    uint8_t sig[8];
    uint32_t board_id;
    uint32_t image_size;
    uint8_t sign_signature[64];
} rid_app_descriptor_t;

static const uint8_t APP_DESCRIPTOR_REV[8] = {
    0x19, 0x75, 0xE2, 0x46, 0x37, 0xF1, 0x2A, 0x43,
};

static const uint8_t *find_bytes(const uint8_t *data,
                                 size_t data_len,
                                 const uint8_t *needle,
                                 size_t needle_len) {
    if (!data || !needle || needle_len == 0 || data_len < needle_len) {
        return NULL;
    }
    for (size_t i = 0; i <= data_len - needle_len; i++) {
        if (memcmp(&data[i], needle, needle_len) == 0) {
            return &data[i];
        }
    }
    return NULL;
}

static bool check_partition_signature(const uint8_t *flash,
                                      uint32_t flash_len,
                                      const rid_app_descriptor_t *desc,
                                      const uint8_t public_key[32]) {
    crypto_check_ctx ctx = {0};
    crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract *)&ctx;
    crypto_check_init(actx, desc->sign_signature, public_key);
    crypto_check_update(actx, flash, flash_len);
    return crypto_check_final(actx) == 0;
}

bool rid_firmware_check_ota_partition(const esp_partition_t *part) {
    if (!part) {
        return false;
    }

    esp_app_desc_t candidate = {0};
    esp_err_t err = esp_ota_get_partition_description(part, &candidate);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "read ESP app descriptor from '%s' failed: %s",
                 part->label,
                 esp_err_to_name(err));
        return false;
    }

    const esp_app_desc_t *running = esp_app_get_description();
    if (!running || strcmp(candidate.project_name, running->project_name) != 0) {
        ESP_LOGE(TAG,
                 "firmware project mismatch: image='%s' local='%s'",
                 candidate.project_name,
                 running ? running->project_name : "unknown");
        return false;
    }
    if (candidate.secure_version < running->secure_version) {
        ESP_LOGE(TAG,
                 "firmware secure version rollback: image=%lu local=%lu",
                 (unsigned long)candidate.secure_version,
                 (unsigned long)running->secure_version);
        return false;
    }

    const void *ptr = NULL;
    esp_partition_mmap_handle_t handle = 0;
    err = esp_partition_mmap(part,
                             0,
                             part->size,
                             ESP_PARTITION_MMAP_DATA,
                             &ptr,
                             &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mmap partition '%s' failed: %s", part->label, esp_err_to_name(err));
        return false;
    }

    uint8_t sig[8] = {0};
    for (uint8_t i = 0; i < sizeof(sig); i++) {
        sig[i] = APP_DESCRIPTOR_REV[sizeof(sig) - 1 - i];
    }

    const uint8_t *flash = (const uint8_t *)ptr;
    const rid_app_descriptor_t *desc =
        (const rid_app_descriptor_t *)find_bytes(flash, part->size, sig, sizeof(sig));
    if (!desc) {
        int8_t lock_level = cfg_get()->lock_level;
        if (lock_level > 0) {
            ESP_LOGE(TAG,
                     "signed app descriptor required at lock level %d",
                     (int)lock_level);
            esp_partition_munmap(handle);
            return false;
        }
        ESP_LOGW(TAG,
                 "accepting native ESP-IDF OTA without legacy signature descriptor: "
                 "project='%s' version='%s' lock=%d",
                 candidate.project_name,
                 candidate.version,
                 (int)lock_level);
        esp_partition_munmap(handle);
        return true;
    }

    uint32_t image_len = (uint32_t)((const uint8_t *)desc - flash);
    if (desc->image_size != image_len) {
        ESP_LOGE(TAG, "bad app descriptor size: descriptor=%lu actual=%lu",
                 (unsigned long)desc->image_size,
                 (unsigned long)image_len);
        if (cfg_get()->lock_level <= 0) {
            ESP_LOGW(TAG,
                     "ignoring malformed legacy descriptor for native ESP-IDF OTA at lock=%d",
                     (int)cfg_get()->lock_level);
            esp_partition_munmap(handle);
            return true;
        }
        esp_partition_munmap(handle);
        return false;
    }

    if (cfg_get()->lock_level != -1 &&
        desc->board_id != 0 &&
        desc->board_id != SERY_RID_CAN_BOARD_ID) {
        ESP_LOGE(TAG, "firmware board id mismatch: image=%lu local=%u",
                 (unsigned long)desc->board_id,
                 SERY_RID_CAN_BOARD_ID);
        esp_partition_munmap(handle);
        return false;
    }

    if (cfg_get()->lock_level == -1 || cfg_no_public_keys()) {
        ESP_LOGW(TAG, "accepting OTA firmware without signature enforcement");
        esp_partition_munmap(handle);
        return true;
    }

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t public_key[32] = {0};
        if (!cfg_get_public_key(i, public_key)) {
            continue;
        }
        if (check_partition_signature(flash, image_len, desc, public_key)) {
            ESP_LOGI(TAG, "firmware signature accepted by public key %u", (unsigned)i);
            esp_partition_munmap(handle);
            return true;
        }
    }

    ESP_LOGE(TAG, "firmware signature check failed");
    esp_partition_munmap(handle);
    return false;
}
