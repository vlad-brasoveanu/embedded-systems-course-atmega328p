#include "filter.h"
#include <math.h>

void Filter_Init(CompFilter_t *f, float alpha)
{
    f->alpha = alpha;
    f->pitch = 0.0f;
    f->roll  = 0.0f;
}

void Filter_Update(CompFilter_t *f,
                   float ax, float ay, float az,
                   float gx, float gy,
                   float dt)
{
    /*
     * Accelerometer-derived angles.
     * pitch: rotation about Y – tilt forward/backward.
     * roll:  rotation about X – tilt left/right.
     *
     * atan2f output is in radians; convert to degrees (* 180/π).
     */
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
    float accel_roll  = atan2f( ay, az)                        * (180.0f / (float)M_PI);

    /* Gyro integration + complementary blend */
    f->pitch = f->alpha * (f->pitch + gx * dt) + (1.0f - f->alpha) * accel_pitch;
    f->roll  = f->alpha * (f->roll  + gy * dt) + (1.0f - f->alpha) * accel_roll;
}
