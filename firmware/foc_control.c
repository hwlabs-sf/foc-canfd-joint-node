/*
 * foc_control.c — see foc_control.h for the interface contract.
 *
 * MIT licence — see ../LICENSE.
 */
#include "foc_control.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SQRT3        1.7320508075688772f
#define SQRT3_BY_2   0.8660254037844386f
#define ONE_BY_SQRT3 0.5773502691896258f

/* ------------------------------------------------------------------ */
/* Transforms                                                          */
/* ------------------------------------------------------------------ */

foc_alphabeta_t foc_clarke(foc_abc_t in)
{
    foc_alphabeta_t out;
    /* Balanced three-wire load: ic is implied by ia + ib + ic = 0. */
    out.alpha = in.phase_a;
    out.beta  = (in.phase_a + 2.0f * in.phase_b) * ONE_BY_SQRT3;
    return out;
}

foc_dq_t foc_park(foc_alphabeta_t in, float theta_e)
{
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    foc_dq_t out;
    out.d =  in.alpha * c + in.beta * s;
    out.q = -in.alpha * s + in.beta * c;
    return out;
}

foc_alphabeta_t foc_inv_park(foc_dq_t in, float theta_e)
{
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);
    foc_alphabeta_t out;
    out.alpha = in.d * c - in.q * s;
    out.beta  = in.d * s + in.q * c;
    return out;
}

/* ------------------------------------------------------------------ */
/* Space-vector PWM                                                    */
/* ------------------------------------------------------------------ */

int foc_svpwm(foc_alphabeta_t v_ab, float vbus, float duty[3])
{
    const float v_limit = (vbus > FOC_VBUS_MIN_V ? vbus : FOC_VBUS_MIN_V) * ONE_BY_SQRT3;
    const float mag2 = v_ab.alpha * v_ab.alpha + v_ab.beta * v_ab.beta;
    int limited = 0;
    float alpha = v_ab.alpha;
    float beta = v_ab.beta;

    if (mag2 > v_limit * v_limit && mag2 > 0.0f) {
        const float scale = v_limit / sqrtf(mag2);
        alpha *= scale;
        beta *= scale;
        limited = 1;
    }

    /* Inverse Clarke into the three phase voltages, then centre them inside
     * the bus so the middle vector sits at 50 % duty. */
    const float va = alpha;
    const float vb = -0.5f * alpha + SQRT3_BY_2 * beta;
    const float vc = -0.5f * alpha - SQRT3_BY_2 * beta;

    const float v_max = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
    const float v_min = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
    const float offset = -0.5f * (v_max + v_min);
    const float inv_bus = 1.0f / (vbus > FOC_VBUS_MIN_V ? vbus : FOC_VBUS_MIN_V);

    duty[0] = 0.5f + (va + offset) * inv_bus;
    duty[1] = 0.5f + (vb + offset) * inv_bus;
    duty[2] = 0.5f + (vc + offset) * inv_bus;

    for (int i = 0; i < 3; ++i) {
        if (duty[i] < 0.0f) { duty[i] = 0.0f; limited = 1; }
        if (duty[i] > 1.0f) { duty[i] = 1.0f; limited = 1; }
    }
    return limited;
}

/* ------------------------------------------------------------------ */
/* PI with clamping anti-windup                                        */
/* ------------------------------------------------------------------ */

void foc_pi_reset(foc_pi_t *pi, float kp, float ki_ts, float out_max)
{
    pi->kp = kp;
    pi->ki_ts = ki_ts;
    pi->out_max = out_max;
    pi->integ = 0.0f;
}

float foc_pi_step(foc_pi_t *pi, float err, float v_applied)
{
    /* Back-calculation: the integrator only accumulates when the controller
     * is not saturated, or when the error pushes it back out of saturation. */
    const float p_term = pi->kp * err;
    const float candidate = pi->integ + pi->ki_ts * err;
    const int saturated = (v_applied >= pi->out_max) || (v_applied <= -pi->out_max);

    if (!(saturated && ((v_applied > 0.0f && err > 0.0f) || (v_applied < 0.0f && err < 0.0f)))) {
        pi->integ = candidate;
    }
    if (pi->integ > pi->out_max) { pi->integ = pi->out_max; }
    if (pi->integ < -pi->out_max) { pi->integ = -pi->out_max; }

    float out = p_term + pi->integ;
    if (out > pi->out_max) { out = pi->out_max; }
    if (out < -pi->out_max) { out = -pi->out_max; }
    return out;
}

/* ------------------------------------------------------------------ */
/* MIT impedance law and thermal derating                              */
/* ------------------------------------------------------------------ */

float foc_mit_torque(const foc_mit_cmd_t *cmd, float p, float v)
{
    return cmd->kp * (cmd->p_des - p) + cmd->kd * (cmd->v_des - v) + cmd->t_ff;
}

float foc_derate_factor(float temp_c)
{
    if (temp_c <= FOC_TEMP_DERATE_START_C) {
        return 1.0f;
    }
    if (temp_c >= FOC_TEMP_DERATE_END_C) {
        return 0.0f;
    }
    return (FOC_TEMP_DERATE_END_C - temp_c) / (FOC_TEMP_DERATE_END_C - FOC_TEMP_DERATE_START_C);
}

float foc_mit_iq_ref(const foc_mit_cmd_t *cmd, float p, float v, float temp_c)
{
    float tau = foc_mit_torque(cmd, p, v) * foc_derate_factor(temp_c);
    float iq = tau / FOC_KT_NM_PER_A;

    if (iq > FOC_I_MAX_A) { iq = FOC_I_MAX_A; }
    if (iq < -FOC_I_MAX_A) { iq = -FOC_I_MAX_A; }
    return iq;
}

/* ------------------------------------------------------------------ */
/* One current-loop tick                                               */
/* ------------------------------------------------------------------ */

void foc_current_loop(const foc_abc_t *i_abc, float theta_e, float vbus,
                      float id_ref, float iq_ref, foc_pi_t *pi_d, foc_pi_t *pi_q,
                      float duty[3], int *limited)
{
    const foc_alphabeta_t i_ab = foc_clarke(*i_abc);
    const foc_dq_t i_dq = foc_park(i_ab, theta_e);

    const float vd = foc_pi_step(pi_d, id_ref - i_dq.d, 0.0f);
    const float vq = foc_pi_step(pi_q, iq_ref - i_dq.q, 0.0f);

    foc_dq_t v_dq;
    v_dq.d = vd;
    v_dq.q = vq;

    const foc_alphabeta_t v_ab = foc_inv_park(v_dq, theta_e);
    *limited = foc_svpwm(v_ab, vbus, duty);
}
