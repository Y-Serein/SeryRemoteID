#ifndef SERY_RID_STATE_H
#define SERY_RID_STATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mavlink/ardupilotmega/mavlink.h"
#include "opendroneid.h"

#ifdef __cplusplus
extern "C" {
#endif

void rid_state_init(void);

void rid_state_update_basic_id(const mavlink_open_drone_id_basic_id_t *msg);
void rid_state_update_location(const mavlink_open_drone_id_location_t *msg);
void rid_state_update_authentication(const mavlink_open_drone_id_authentication_t *msg);
void rid_state_update_self_id(const mavlink_open_drone_id_self_id_t *msg);
void rid_state_update_system(const mavlink_open_drone_id_system_t *msg);
void rid_state_update_system_update(const mavlink_open_drone_id_system_update_t *msg);
void rid_state_update_operator_id(const mavlink_open_drone_id_operator_id_t *msg);

bool rid_state_build_uas_data(ODID_UAS_Data *out, char *reason, size_t reason_size);
bool rid_state_arm_status(char *reason, size_t reason_size);
bool rid_state_have_location(void);

#ifdef __cplusplus
}
#endif

#endif
