#ifndef SERY_RID_CFG_H
#define SERY_RID_CFG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char b64_key[65];
} rid_public_key_t;

typedef enum {
    RID_PARAM_NONE = 0,
    RID_PARAM_UINT8,
    RID_PARAM_INT8,
    RID_PARAM_UINT32,
    RID_PARAM_FLOAT,
    RID_PARAM_CHAR20,
    RID_PARAM_CHAR64,
} rid_param_type_t;

typedef struct {
    char name[17];
    rid_param_type_t type;
    uint16_t flags;
    uint8_t min_len;
    float default_value;
    float min_value;
    float max_value;
} rid_param_info_t;

typedef struct {
    const rid_param_info_t *info;
    void *value;
} rid_param_t;

typedef struct {
    int8_t lock_level;
    uint8_t can_node;
    uint8_t can_terminate;
    uint32_t uart_baud;
    int uart_tx_gpio;
    int uart_rx_gpio;
    uint8_t wifi_channel;
    float wifi_power_dbm;
    float wifi_nan_rate_hz;
    float wifi_beacon_rate_hz;
    float bt4_rate_hz;
    float bt4_power_dbm;
    float bt5_rate_hz;
    float bt5_power_dbm;
    bool broadcast_powerup;
    uint8_t webserver_enable;
    uint8_t mavlink_sysid;
    uint8_t options;
    uint8_t ua_type;
    uint8_t uas_id_type;
    char uas_id[21];
    uint8_t ua_type_2;
    uint8_t uas_id_type_2;
    char uas_id_2[21];
    char wifi_ssid[33];
    char wifi_password[21];
    uint8_t to_factory_defaults;
    uint8_t done_init;
    rid_public_key_t public_keys[5];
} rid_config_t;

#define RID_PARAM_FLAG_NONE     0u
#define RID_PARAM_FLAG_PASSWORD (1u << 0)
#define RID_PARAM_FLAG_HIDDEN   (1u << 1)

#define RID_OPTIONS_FORCE_ARM_OK                    (1u << 0)
#define RID_OPTIONS_DONT_SAVE_BASIC_ID_TO_PARAMS    (1u << 1)
#define RID_OPTIONS_PRINT_RID_MAVLINK               (1u << 2)

void cfg_init(void);
const rid_config_t *cfg_get(void);
bool cfg_have_basic_id(void);
bool cfg_have_basic_id_2(void);

const rid_param_t *cfg_param_find(const char *name);
const rid_param_t *cfg_param_find_by_index(uint16_t index);
const rid_param_t *cfg_param_find_float_by_index(uint16_t index);
uint16_t cfg_param_count_float(void);
int16_t cfg_param_index_float(const rid_param_t *param);
bool cfg_param_get_as_float(const rid_param_t *param, float *value);
bool cfg_param_set_as_float(const rid_param_t *param, float value);
bool cfg_param_set_by_name_string(const char *name, const char *value);
bool cfg_param_set_by_name_uint8(const char *name, uint8_t value);
bool cfg_param_set_by_name_char64(const char *name, const char *value);
const char *cfg_param_get_string(const rid_param_t *param);

bool cfg_get_public_key(uint8_t index, uint8_t key[32]);
bool cfg_set_public_key(uint8_t index, const uint8_t key[32]);
bool cfg_remove_public_key(uint8_t index);
bool cfg_no_public_keys(void);

#ifdef __cplusplus
}
#endif

#endif
