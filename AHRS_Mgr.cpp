#include <math.h>
#include "AHRS_Mgr.h"

float invSqrt(float x);

const float M_PI_F = 3.14159265358979323846f;

const float TWO_KP_DEF = (2.0f * 0.4f);   // 2 * proportional gain
const float TWO_KI_DEF = (2.0f * 0.001f); // 2 * integral gain

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f; // quaternion of sensor frame relative to auxiliary frame

float twoKp = TWO_KP_DEF; // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF; // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f; // integral error terms scaled by Ki

static float gravX, gravY, gravZ; // Unit vector in the estimated gravity direction

void AHRS_Mgr_update_top(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    AHRS_Mgr_update(gx, gy, gz, ax, ay, az, dt);
    AHRS_Mgr_estimatedGravityDirection(&gravX, &gravY, &gravZ);

    /* if (!isCalibrated) {
    baseZacc = sensfusion6GetAccZ(ax, ay, az);
    isCalibrated = true;   */
}


// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void AHRS_Mgr_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx = gx * M_PI_F / 180;
    gy = gy * M_PI_F / 180;
    gz = gz * M_PI_F / 180;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = qx * qz - qw * qy;
        halfvy = qw * qx + qy * qz;
        halfvz = qw * qw - 0.5f + qz * qz;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = qw;
    qb = qx;
    qc = qy;
    qw += (-qb * gx - qc * gy - qz * gz);
    qx += (qa * gx + qc * gz - qz * gy);
    qy += (qa * gy - qb * gz + qz * gx);
    qz += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= recipNorm;
    qx *= recipNorm;
    qy *= recipNorm;
    qz *= recipNorm;
}

void AHRS_Mgr_estimatedGravityDirection(float *gx, float *gy, float *gz)
{
    *gx = 2 * (qx * qz - qw * qy);
    *gy = 2 * (qw * qx + qy * qz);
    *gz = qw * qw - qx * qx - qy * qy + qz * qz;
}

void AHRS_Mgr_GetEulerRPY(float *roll, float *pitch, float *yaw)
{
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;

    if (gx > 1)
        gx = 1;
    if (gx < -1)
        gx = -1;

    *yaw = atan2f(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / M_PI_F;
    *pitch = asinf(gx) * 180 / M_PI_F; // Pitch seems to be inverted
    *roll = atan2f(gy, gz) * 180 / M_PI_F;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
