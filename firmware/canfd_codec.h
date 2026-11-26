/*
 * canfd_codec.h — bit-packed CAN-FD frames for the joint node.
 *
 * Two frame types, both daisy-chained on the same bus:
 *
 *   MIT command  (8 bytes, ID 0x100 + node)      host  -> joint
 *   Joint status (12 bytes, ID 0x200 + node)     joint -> host
 *
 * All fields are little-endian, fixed point, and saturate at the wire limits
 * rather than wrapping. Nothing here depends on the STM32 HAL: the caller
 * hands in a byte buffer and a length.
 *
 * MIT licence — see ../LICENSE.
 */
#ifndef CANFD_CODEC_H
#define CANFD_CODEC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_CMD_ID_BASE     0x100u
#define CANFD_STATUS_ID_BASE  0x200u
#define CANFD_CMD_LEN         8u
#define CANFD_STATUS_LEN      12u

/* Fixed-point scales (value = raw * scale). */
#define CANFD_POS_SCALE       0.001f    /* rad per LSB, int16  */
#define CANFD_VEL_SCALE       0.01f     /* rad/s per LSB, int16 */
#define CANFD_KP_SCALE        0.5f      /* Nm/rad per LSB, uint8 */
#define CANFD_KD_SCALE        0.01f     /* Nm/(rad/s) per LSB, uint8 */
#define CANFD_TFF_SCALE       0.001f    /* Nm per LSB, int16  */
#define CANFD_CUR_SCALE       0.01f     /* A per LSB, int16   */
#define CANFD_VBUS_SCALE      0.25f     /* V per LSB, uint16  */

typedef struct {
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
} canfd_mit_cmd_t;

typedef struct {
    float p;
    float v;
    float iq;
    float id;
    float vbus;
    int16_t temp_c;
    uint8_t fault;
} canfd_status_t;

/* Returns the command CAN ID for a node address (1..63). */
uint32_t canfd_cmd_id(uint8_t node);
uint32_t canfd_status_id(uint8_t node);

/* Encode/decode an 8-byte MIT command. Return 0 on success, -1 if the buffer
 * is shorter than CANFD_CMD_LEN. */
int canfd_encode_cmd(const canfd_mit_cmd_t *cmd, uint8_t out[CANFD_CMD_LEN]);
int canfd_decode_cmd(const uint8_t in[CANFD_CMD_LEN], canfd_mit_cmd_t *out);

/* Encode/decode a 12-byte status frame. */
int canfd_encode_status(const canfd_status_t *st, uint8_t out[CANFD_STATUS_LEN]);
int canfd_decode_status(const uint8_t in[CANFD_STATUS_LEN], canfd_status_t *out);

/* CAN-FD DLC code for a payload length: 0..8 -> same, 12 -> 9, 16 -> 10,
 * 20 -> 11, 24 -> 12, 32 -> 13, 48 -> 14, 64 -> 15. Returns -1 if invalid. */
int8_t canfd_dlc_for_len(uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* CANFD_CODEC_H */
