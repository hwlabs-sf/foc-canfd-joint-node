/*
 * canfd_codec.c — see canfd_codec.h for the wire format.
 *
 * MIT licence — see ../LICENSE.
 */
#include "canfd_codec.h"

#include <math.h>

/* ---- fixed-point helpers (saturating, round-to-nearest) ------------ */

static int16_t sat_i16(float x)
{
    if (x >= 32767.0f) { return 32767; }
    if (x <= -32768.0f) { return -32768; }
    return (int16_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));
}

static uint16_t sat_u16(float x)
{
    if (x <= 0.0f) { return 0u; }
    if (x >= 65535.0f) { return 65535u; }
    return (uint16_t)(x + 0.5f);
}

static uint8_t sat_u8(float x)
{
    if (x <= 0.0f) { return 0u; }
    if (x >= 255.0f) { return 255u; }
    return (uint8_t)(x + 0.5f);
}

static void put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ---- IDs ---------------------------------------------------------- */

uint32_t canfd_cmd_id(uint8_t node)
{
    return (uint32_t)CANFD_CMD_ID_BASE + (uint32_t)(node & 0x3Fu);
}

uint32_t canfd_status_id(uint8_t node)
{
    return (uint32_t)CANFD_STATUS_ID_BASE + (uint32_t)(node & 0x3Fu);
}

/* ---- MIT command: 8 bytes -----------------------------------------
 *  [0:1] p_des   int16, 1 mm rad
 *  [2:3] v_des   int16, 0.01 rad/s
 *  [4]   kp      uint8, 0.5 Nm/rad
 *  [5]   kd      uint8, 0.01 Nm/(rad/s)
 *  [6:7] t_ff    int16, 1 mm Nm
 */
int canfd_encode_cmd(const canfd_mit_cmd_t *cmd, uint8_t out[CANFD_CMD_LEN])
{
    if (!cmd || !out) {
        return -1;
    }
    put_i16(&out[0], sat_i16(cmd->p_des / CANFD_POS_SCALE));
    put_i16(&out[2], sat_i16(cmd->v_des / CANFD_VEL_SCALE));
    out[4] = sat_u8(cmd->kp / CANFD_KP_SCALE);
    out[5] = sat_u8(cmd->kd / CANFD_KD_SCALE);
    put_i16(&out[6], sat_i16(cmd->t_ff / CANFD_TFF_SCALE));
    return 0;
}

int canfd_decode_cmd(const uint8_t in[CANFD_CMD_LEN], canfd_mit_cmd_t *out)
{
    if (!in || !out) {
        return -1;
    }
    out->p_des = (float)get_i16(&in[0]) * CANFD_POS_SCALE;
    out->v_des = (float)get_i16(&in[2]) * CANFD_VEL_SCALE;
    out->kp    = (float)in[4] * CANFD_KP_SCALE;
    out->kd    = (float)in[5] * CANFD_KD_SCALE;
    out->t_ff  = (float)get_i16(&in[6]) * CANFD_TFF_SCALE;
    return 0;
}

/* ---- status: 12 bytes ---------------------------------------------
 *  [0:1]  p      int16, 1 mm rad
 *  [2:3]  v      int16, 0.01 rad/s
 *  [4:5]  iq     int16, 0.01 A
 *  [6:7]  id     int16, 0.01 A
 *  [8:9]  vbus   uint16, 0.25 V
 *  [10]   temp   int8, 1 C
 *  [11]   fault  uint8 bitfield
 */
int canfd_encode_status(const canfd_status_t *st, uint8_t out[CANFD_STATUS_LEN])
{
    if (!st || !out) {
        return -1;
    }
    put_i16(&out[0], sat_i16(st->p / CANFD_POS_SCALE));
    put_i16(&out[2], sat_i16(st->v / CANFD_VEL_SCALE));
    put_i16(&out[4], sat_i16(st->iq / CANFD_CUR_SCALE));
    put_i16(&out[6], sat_i16(st->id / CANFD_CUR_SCALE));
    put_u16(&out[8], sat_u16(st->vbus / CANFD_VBUS_SCALE));

    if (st->temp_c > 127) { out[10] = 127u; }
    else if (st->temp_c < -128) { out[10] = (uint8_t)(-128 & 0xFF); }
    else { out[10] = (uint8_t)(st->temp_c & 0xFF); }

    out[11] = st->fault;
    return 0;
}

int canfd_decode_status(const uint8_t in[CANFD_STATUS_LEN], canfd_status_t *out)
{
    if (!in || !out) {
        return -1;
    }
    out->p       = (float)get_i16(&in[0]) * CANFD_POS_SCALE;
    out->v       = (float)get_i16(&in[2]) * CANFD_VEL_SCALE;
    out->iq      = (float)get_i16(&in[4]) * CANFD_CUR_SCALE;
    out->id      = (float)get_i16(&in[6]) * CANFD_CUR_SCALE;
    out->vbus    = (float)get_u16(&in[8]) * CANFD_VBUS_SCALE;
    out->temp_c  = (int16_t)(int8_t)in[10];
    out->fault   = in[11];
    return 0;
}

/* ---- DLC mapping --------------------------------------------------- */

int8_t canfd_dlc_for_len(uint8_t len)
{
    static const uint8_t fd_dlc[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u
    };
    if (len <= 8u) {
        return (int8_t)fd_dlc[len];
    }
    if (len <= 12u)  { return 9; }
    if (len <= 16u)  { return 10; }
    if (len <= 20u)  { return 11; }
    if (len <= 24u)  { return 12; }
    if (len <= 32u)  { return 13; }
    if (len <= 48u)  { return 14; }
    if (len <= 64u)  { return 15; }
    return -1;
}
