#include "security/rid_secure.h"

#include <string.h>

#include "cfg.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "monocypher.h"

static uint8_t s_session_key[8];

static uint64_t crc64_words(const uint32_t *data, uint16_t num_words) {
    const uint64_t poly = 0x42F0E1EBA9EA3693ULL;
    uint64_t crc = ~(0ULL);
    while (num_words--) {
        uint32_t value = *data++;
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t byte = ((uint8_t *)&value)[j];
            crc ^= (uint64_t)byte << 56u;
            for (uint8_t i = 0; i < 8; i++) {
                if (crc & (1ULL << 63u)) {
                    crc = (uint64_t)(crc << 1u) ^ poly;
                } else {
                    crc = (uint64_t)(crc << 1u);
                }
            }
        }
    }
    return crc ^ ~(0ULL);
}

void rid_secure_make_session_key(uint8_t key[8]) {
    struct {
        uint32_t time_us;
        uint8_t mac[8];
        uint32_t random_word;
    } data = {0};

    esp_efuse_mac_get_default(data.mac);
    data.time_us = (uint32_t)esp_timer_get_time();
    data.random_word = esp_random();
    uint64_t crc = crc64_words((const uint32_t *)&data, sizeof(data) / sizeof(uint32_t));
    memcpy(s_session_key, &crc, sizeof(s_session_key));
    if (key) {
        memcpy(key, s_session_key, sizeof(s_session_key));
    }
}

const uint8_t *rid_secure_session_key(void) {
    return s_session_key;
}

bool rid_secure_check_signature(uint8_t sig_length,
                                uint8_t data_len,
                                uint32_t sequence,
                                uint32_t operation,
                                const uint8_t *data) {
    if (cfg_no_public_keys()) {
        return true;
    }
    if (!data || sig_length != 64) {
        return false;
    }

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t key[32] = {0};
        if (!cfg_get_public_key(i, key)) {
            continue;
        }

        crypto_check_ctx ctx = {0};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract *)&ctx;
        crypto_check_init(actx, &data[data_len], key);
        crypto_check_update(actx, (const uint8_t *)&sequence, sizeof(sequence));
        crypto_check_update(actx, (const uint8_t *)&operation, sizeof(operation));
        crypto_check_update(actx, data, data_len);
        if (operation != SECURE_COMMAND_GET_SESSION_KEY &&
            operation != SECURE_COMMAND_GET_REMOTEID_SESSION_KEY) {
            crypto_check_update(actx, s_session_key, sizeof(s_session_key));
        }
        if (crypto_check_final(actx) == 0) {
            return true;
        }
    }

    return false;
}
