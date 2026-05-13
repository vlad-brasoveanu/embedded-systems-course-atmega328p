#include "madgwick.h"
#include <math.h>

#define DEG_TO_RAD  (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG  (180.0f / 3.14159265358979323846f)

/* Fast inverse square root – on AVR just wraps 1/sqrtf */
static inline float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void Madgwick_Init(Madgwick_t *f, float beta)
{
    f->beta = beta;
    f->q0   = 1.0f;
    f->q1   = 0.0f;
    f->q2   = 0.0f;
    f->q3   = 0.0f;
}

/* -------------------------------------------------------------------------
 * 9-DOF update (Madgwick AHRS, full magnetometer fusion)
 * Gyroscope input in deg/s; converted to rad/s at the top.
 * ------------------------------------------------------------------------- */
void Madgwick_Update9DOF(Madgwick_t *f,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float hx, hy;
    float _2bx, _2bz, _4bx, _4bz;

    /* Convert gyroscope to rad/s */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    /* Ignore magnetometer if zero-vector */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        Madgwick_Update6DOF(f, gx * RAD_TO_DEG, gy * RAD_TO_DEG,
                              gz * RAD_TO_DEG, ax, ay, az, dt);
        return;
    }

    /* Rate of change from gyroscope */
    q_dot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    q_dot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    q_dot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    q_dot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    /* Normalise accelerometer */
    recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
    ax *= recip_norm; ay *= recip_norm; az *= recip_norm;

    /* Normalise magnetometer */
    recip_norm = inv_sqrt(mx*mx + my*my + mz*mz);
    mx *= recip_norm; my *= recip_norm; mz *= recip_norm;

    /* Precomputed products */
    float _2q0mx = 2.0f*q0*mx, _2q0my = 2.0f*q0*my, _2q0mz = 2.0f*q0*mz;
    float _2q1mx = 2.0f*q1*mx;
    float _2q0   = 2.0f*q0,  _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    float _2q0q2 = 2.0f*q0*q2, _2q2q3 = 2.0f*q2*q3;
    float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    /* Reference direction of Earth's magnetic field in navigation frame */
    hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1
         + 2.0f*q1*my*q2 + 2.0f*q1*mz*q3 - mx*q2q2 - mx*q3q3;
    hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2
         - my*q1q1 + my*q2q2 + 2.0f*q2*mz*q3 - my*q3q3;
    _2bx = sqrtf(hx*hx + hy*hy);
    _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3
            - mz*q1q1 + 2.0f*q2*my*q3 - mz*q2q2 + mz*q3q3;
    _4bx = 2.0f*_2bx;
    _4bz = 2.0f*_2bz;

    /* Gradient descent algorithm corrective step */
    s0 = -_2q2*(2.0f*q1q3 - _2q0q2 - ax)
         + _2q1*(2.0f*q0q1 + _2q2q3 - ay)
         - _2bz*q2*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
         + (-_2bx*q3+_2bz*q1)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
         + _2bx*q2*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);

    s1 =  _2q3*(2.0f*q1q3 - _2q0q2 - ax)
         + _2q0*(2.0f*q0q1 + _2q2q3 - ay)
         - 4.0f*q1*(1.0f-2.0f*q1q1-2.0f*q2q2-az)
         + _2bz*q3*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
         + (_2bx*q2+_2bz*q0)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
         + (_2bx*q3-_4bz*q1)*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);

    s2 = -_2q0*(2.0f*q1q3 - _2q0q2 - ax)
         + _2q3*(2.0f*q0q1 + _2q2q3 - ay)
         - 4.0f*q2*(1.0f-2.0f*q1q1-2.0f*q2q2-az)
         + (-_4bx*q2-_2bz*q0)*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
         + (_2bx*q1+_2bz*q3)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
         + (_2bx*q0-_4bz*q2)*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);

    s3 =  _2q1*(2.0f*q1q3 - _2q0q2 - ax)
         + _2q2*(2.0f*q0q1 + _2q2q3 - ay)
         + (-_4bx*q3+_2bz*q1)*(_2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2) - mx)
         + (-_2bx*q0+_2bz*q2)*(_2bx*(q1q2-q0q3) + _2bz*(q0q1+q2q3) - my)
         + _2bx*q1*(_2bx*(q0q2+q1q3) + _2bz*(0.5f-q1q1-q2q2) - mz);

    /* Normalise gradient */
    recip_norm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recip_norm; s1 *= recip_norm; s2 *= recip_norm; s3 *= recip_norm;

    /* Feedback step */
    q_dot1 -= f->beta * s0;
    q_dot2 -= f->beta * s1;
    q_dot3 -= f->beta * s2;
    q_dot4 -= f->beta * s3;

    /* Integrate */
    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    /* Normalise quaternion */
    recip_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    f->q0 = q0 * recip_norm;
    f->q1 = q1 * recip_norm;
    f->q2 = q2 * recip_norm;
    f->q3 = q3 * recip_norm;
}

/* -------------------------------------------------------------------------
 * 6-DOF update (accel + gyro only, no magnetometer)
 * ------------------------------------------------------------------------- */
void Madgwick_Update6DOF(Madgwick_t *f,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt)
{
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;

    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    q_dot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    q_dot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    q_dot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    q_dot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    /* Only apply gradient step if accelerometer vector is non-zero */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm; ay *= recip_norm; az *= recip_norm;

        /* Gradient of objective function for accel measurement */
        s0 = 4.0f*q0*q2*q2 + 2.0f*q2*ax + 4.0f*q0*q1*q1 - 2.0f*q1*ay;
        s1 = 4.0f*q1*q3*q3 - 2.0f*q3*ax + 4.0f*q0*q0*q1
             - 2.0f*q0*ay - 4.0f*q1 + 8.0f*q1*q1*q1 + 8.0f*q1*q2*q2 + 4.0f*q1*az;
        s2 = 4.0f*q0*q0*q2 + 2.0f*q0*ax + 4.0f*q2*q3*q3
             - 2.0f*q3*ay - 4.0f*q2 + 8.0f*q2*q1*q1 + 8.0f*q2*q2*q2 + 4.0f*q2*az;
        s3 = 4.0f*q1*q1*q3 - 2.0f*q1*ax + 4.0f*q2*q2*q3 - 2.0f*q2*ay;

        recip_norm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recip_norm; s1 *= recip_norm; s2 *= recip_norm; s3 *= recip_norm;

        q_dot1 -= f->beta * s0;
        q_dot2 -= f->beta * s1;
        q_dot3 -= f->beta * s2;
        q_dot4 -= f->beta * s3;
    }

    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    recip_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    f->q0 = q0 * recip_norm;
    f->q1 = q1 * recip_norm;
    f->q2 = q2 * recip_norm;
    f->q3 = q3 * recip_norm;
}

/* -------------------------------------------------------------------------
 * Euler angle extraction from quaternion (ZYX / aerospace convention)
 * ------------------------------------------------------------------------- */
float Madgwick_GetPitch(const Madgwick_t *f)
{
    /* pitch = arcsin(-2(q1·q3 - q0·q2)) */
    return asinf(-2.0f*(f->q1*f->q3 - f->q0*f->q2)) * RAD_TO_DEG;
}

float Madgwick_GetRoll(const Madgwick_t *f)
{
    /* roll = atan2(2(q0·q1 + q2·q3), q0²-q1²-q2²+q3²) */
    return atan2f(2.0f*(f->q0*f->q1 + f->q2*f->q3),
                  f->q0*f->q0 - f->q1*f->q1 - f->q2*f->q2 + f->q3*f->q3)
           * RAD_TO_DEG;
}

float Madgwick_GetYaw(const Madgwick_t *f)
{
    /* yaw = atan2(2(q1·q2 + q0·q3), q0²+q1²-q2²-q3²) */
    return atan2f(2.0f*(f->q1*f->q2 + f->q0*f->q3),
                  f->q0*f->q0 + f->q1*f->q1 - f->q2*f->q2 - f->q3*f->q3)
           * RAD_TO_DEG;
}
