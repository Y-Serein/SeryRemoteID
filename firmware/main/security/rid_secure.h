#ifndef SERY_RID_SECURE_H
#define SERY_RID_SECURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rid_secure_make_session_key(uint8_t key[8]);
bool rid_secure_check_signature(uint8_t sig_length,
                                uint8_t data_len,
                                uint32_t sequence,
                                uint32_t operation,
                                const uint8_t *data);
const uint8_t *rid_secure_session_key(void);

#ifdef __cplusplus
}
#endif

#endif
