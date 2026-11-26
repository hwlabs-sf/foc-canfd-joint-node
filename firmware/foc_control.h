/*
 * foc_control.h — field-oriented control core for the 42 mm CAN-FD joint node.
 *
 * Target: STM32G431, TIM1 centre-aligned PWM at 20 kHz, injected ADC on the
 * low-side Kelvin shunts. Everything here is ISO C99, no heap, no libc beyond
 * <math.h>, and safe to call from the current-loop ISR.
 *
 * MIT licence — see ../LICENSE.
 */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- machine constants (rev A: 14 pole pairs, 85 mNm/A) ------------------ */
#define FOC_POLE_PAIRS            7.0f
#define FOC_KT_NM_PER_A           0.085f
#define FOC_I_MAX_A               15.0f
#define FOC_VBUS_MIN_V            24.0f
#define FOC_VBUS_MAX_V            60.0f
#define FOC_PWM_HZ                20000.0f
#define FOC_TEMP_DERATE_START_C   85.0f
#define FOC_TEMP_DERATE_END_C     110.0f
#define FOC_PI_KI_TS              0.0035f   /* Ki * Ts, pre-multiplied */
#define FOC_PI_MAX_V              13.8f     /* per-axis voltage clamp, 48 V bus */

typedef struct {
    float kp;
    float ki_ts;
    float integ;
    float out_max;
} foc_pi_t;

typedef struct {
    float phase_a;
    float phase_b;
    float phase_c;
} foc_abc_t;

typedef struct {
    float alpha;
    float beta;
} foc_alphabeta_t;

typedef struct {
    float d;
    float q;
} foc_dq_t;

typedef struct {
    float p_des;    /* rad   */
    float v_des;    /* rad/s */
    float kp;       /* Nm/rad   */
    float kd;       /* Nm/(rad/s) */
    float t_ff;     /* Nm */
} foc_mit_cmd_t;

/* Transforms ------------------------------------------------------------- */
foc_alphabeta_t foc_clarke(foc_abc_t in);
foc_dq_t        foc_park(foc_alphabeta_t in, float theta_e);
foc_alphabeta_t foc_inv_park(foc_dq_t in, float theta_e);

/* Space-vector PWM -------------------------------------------------------
 * Writes centre-aligned duties in [0, 1] for the three half bridges and
 * returns 1 when the voltage request had to be limited (over-modulation or
 * bus under-voltage), 0 otherwise.
 */
int foc_svpwm(foc_alphabeta_t v_ab, float vbus, float duty[3]);

/* Current regulators ----------------------------------------------------- */
void  foc_pi_reset(foc_pi_t *pi, float kp, float ki_ts, float out_max);
float foc_pi_step(foc_pi_t *pi, float err, float v_applied);

/* MIT impedance law ------------------------------------------------------
 * tau = kp * (p_des - p) + kd * (v_des - v) + t_ff
 */
float foc_mit_torque(const foc_mit_cmd_t *cmd, float p, float v);

/* 1.0 below the derating knee, falling linearly to 0 at the cut-off. */
float foc_derate_factor(float temp_c);

/* Full command path: MIT law -> torque -> iq reference, with the derating
 * factor and the phase-current ceiling applied. */
float foc_mit_iq_ref(const foc_mit_cmd_t *cmd, float p, float v, float temp_c);

/* One current-loop tick: returns the duties to load into TIM1 CCRx. */
void foc_current_loop(const foc_abc_t *i_abc, float theta_e, float vbus,
                      float id_ref, float iq_ref, foc_pi_t *pi_d, foc_pi_t *pi_q,
                      float duty[3], int *limited);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_H */
