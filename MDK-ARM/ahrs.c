#include "ahrs.h"

/* Calculate 1/sqrt(x) with magic number support */
static
float oneOverSqrt(float x) {
    return 1.0f / (float) sqrt(x);
}

static
void calculateAngles(AHRS_t* AHRS) {
    AHRS->Roll = (float) atan2(AHRS->_q0 * AHRS->_q1 + AHRS->_q2 * AHRS->_q3, 0.5f - AHRS->_q1 * AHRS->_q1 - AHRS->_q2 * AHRS->_q2);
    AHRS->Pitch = (float) asin(-2.0f * (AHRS->_q1 * AHRS->_q3 - AHRS->_q0 * AHRS->_q2));
    AHRS->Yaw = (float) atan2(AHRS->_q1 * AHRS->_q2 + AHRS->_q0 * AHRS->_q3, 0.5f - AHRS->_q2 * AHRS->_q2 - AHRS->_q3 * AHRS->_q3);

    /* Calculate degrees and remove inclination */
    AHRS->Roll *= AHRS_RAD2DEG(1);
    AHRS->Pitch *= AHRS_RAD2DEG(1);
    AHRS->Yaw = AHRS->Yaw * AHRS_RAD2DEG(1) - AHRS->Inclination;
    
    /* Check values because of inclination */
    if (AHRS->Yaw < -180) {
        AHRS->Yaw = 180.0f - (-180.0f - AHRS->Yaw);
    } else if (AHRS->Yaw > 180) {
        AHRS->Yaw = -180.0f - (180.0f - AHRS->Yaw);
    }
}

void AHRS_Init(AHRS_t* AHRS, float beta, float sampleRate, float inclination) {
    AHRS->_beta = beta;
    AHRS->_sampleRate = 1 / sampleRate;
    AHRS->Inclination = inclination;
    
    AHRS->_q0 = 1.0f;
    AHRS->_q1 = 0.0f;
    AHRS->_q2 = 0.0f;
    AHRS->_q3 = 0.0f;
}

void AHRS_UpdateAHRS(AHRS_t* AHRS, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /* Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation) */
    if ((mx != 0.0f) || (my != 0.0f) || (mz != 0.0f)) {        
        /* Rate of change of quaternion from gyroscope */
        qDot1 = 0.5f * (-AHRS->_q1 * gx - AHRS->_q2 * gy - AHRS->_q3 * gz);
        qDot2 = 0.5f * (AHRS->_q0 * gx + AHRS->_q2 * gz - AHRS->_q3 * gy);
        qDot3 = 0.5f * (AHRS->_q0 * gy - AHRS->_q1 * gz + AHRS->_q3 * gx);
        qDot4 = 0.5f * (AHRS->_q0 * gz + AHRS->_q1 * gy - AHRS->_q2 * gx);

        /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            /* Normalise accelerometer measurement */
            recipNorm = oneOverSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            /* Normalise magnetometer measurement */
            recipNorm = oneOverSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            /* Auxiliary variables to avoid repeated arithmetic */
            _2q0mx = 2.0f * AHRS->_q0 * mx;
            _2q0my = 2.0f * AHRS->_q0 * my;
            _2q0mz = 2.0f * AHRS->_q0 * mz;
            _2q1mx = 2.0f * AHRS->_q1 * mx;
            _2q0 = 2.0f * AHRS->_q0;
            _2q1 = 2.0f * AHRS->_q1;
            _2q2 = 2.0f * AHRS->_q2;
            _2q3 = 2.0f * AHRS->_q3;
            _2q0q2 = 2.0f * AHRS->_q0 * AHRS->_q2;
            _2q2q3 = 2.0f * AHRS->_q2 * AHRS->_q3;
            q0q0 = AHRS->_q0 * AHRS->_q0;
            q0q1 = AHRS->_q0 * AHRS->_q1;
            q0q2 = AHRS->_q0 * AHRS->_q2;
            q0q3 = AHRS->_q0 * AHRS->_q3;
            q1q1 = AHRS->_q1 * AHRS->_q1;
            q1q2 = AHRS->_q1 * AHRS->_q2;
            q1q3 = AHRS->_q1 * AHRS->_q3;
            q2q2 = AHRS->_q2 * AHRS->_q2;
            q2q3 = AHRS->_q2 * AHRS->_q3;
            q3q3 = AHRS->_q3 * AHRS->_q3;

            /* Reference direction of Earth's magnetic field */
            hx = mx * q0q0 - _2q0my * AHRS->_q3 + _2q0mz * AHRS->_q2 + mx * q1q1 + _2q1 * my * AHRS->_q2 + _2q1 * mz * AHRS->_q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * AHRS->_q3 + my * q0q0 - _2q0mz * AHRS->_q1 + _2q1mx * AHRS->_q2 - my * q1q1 + my * q2q2 + _2q2 * mz * AHRS->_q3 - my * q3q3;
            _2bx = (float) sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * AHRS->_q2 + _2q0my * AHRS->_q1 + mz * q0q0 + _2q1mx * AHRS->_q3 - mz * q1q1 + _2q2 * my * AHRS->_q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            /* Gradient decent algorithm corrective step */
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * AHRS->_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRS->_q3 + _2bz * AHRS->_q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRS->_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRS->_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * AHRS->_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRS->_q2 + _2bz * AHRS->_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRS->_q3 - _4bz * AHRS->_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * AHRS->_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * AHRS->_q2 - _2bz * AHRS->_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * AHRS->_q1 + _2bz * AHRS->_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * AHRS->_q0 - _4bz * AHRS->_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * AHRS->_q3 + _2bz * AHRS->_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * AHRS->_q0 + _2bz * AHRS->_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * AHRS->_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            
            /* normalise step magnitude */
            recipNorm = oneOverSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            /* Apply feedback step */
            qDot1 -= AHRS->_beta * s0;
            qDot2 -= AHRS->_beta * s1;
            qDot3 -= AHRS->_beta * s2;
            qDot4 -= AHRS->_beta * s3;
        }

        /* Integrate rate of change of quaternion to yield quaternion */
        AHRS->_q0 += qDot1 * AHRS->_sampleRate;
        AHRS->_q1 += qDot2 * AHRS->_sampleRate;
        AHRS->_q2 += qDot3 * AHRS->_sampleRate;
        AHRS->_q3 += qDot4 * AHRS->_sampleRate;

        /* Normalise quaternion */
        recipNorm = oneOverSqrt(AHRS->_q0 * AHRS->_q0 + AHRS->_q1 * AHRS->_q1 + AHRS->_q2 * AHRS->_q2 + AHRS->_q3 * AHRS->_q3);
        AHRS->_q0 *= recipNorm;
        AHRS->_q1 *= recipNorm;
        AHRS->_q2 *= recipNorm;
        AHRS->_q3 *= recipNorm;
    } else {
        /* Update IMU algorithm */
        AHRS_UpdateIMU(AHRS, gx, gy, gz, ax, ay, az);
    }
    
    /* Calculate new angles */
    calculateAngles(AHRS);
}

void AHRS_UpdateIMU(AHRS_t* AHRS, float ax, float ay, float az, float gx, float gy, float gz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-AHRS->_q1 * gx - AHRS->_q2 * gy - AHRS->_q3 * gz);
    qDot2 = 0.5f * (AHRS->_q0 * gx + AHRS->_q2 * gz - AHRS->_q3 * gy);
    qDot3 = 0.5f * (AHRS->_q0 * gy - AHRS->_q1 * gz + AHRS->_q3 * gx);
    qDot4 = 0.5f * (AHRS->_q0 * gz + AHRS->_q1 * gy - AHRS->_q2 * gx);

    /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {

        /* Normalise accelerometer measurement */
        recipNorm = oneOverSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        _2q0 = 2.0f * AHRS->_q0;
        _2q1 = 2.0f * AHRS->_q1;
        _2q2 = 2.0f * AHRS->_q2;
        _2q3 = 2.0f * AHRS->_q3;
        _4q0 = 4.0f * AHRS->_q0;
        _4q1 = 4.0f * AHRS->_q1;
        _4q2 = 4.0f * AHRS->_q2;
        _8q1 = 8.0f * AHRS->_q1;
        _8q2 = 8.0f * AHRS->_q2;
        q0q0 = AHRS->_q0 * AHRS->_q0;
        q1q1 = AHRS->_q1 * AHRS->_q1;
        q2q2 = AHRS->_q2 * AHRS->_q2;
        q3q3 = AHRS->_q3 * AHRS->_q3;

        /* Gradient decent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * AHRS->_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * AHRS->_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * AHRS->_q3 - _2q1 * ax + 4.0f * q2q2 * AHRS->_q3 - _2q2 * ay;
        
        /* Normalise step magnitude */
        recipNorm = oneOverSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot1 -= AHRS->_beta * s0;
        qDot2 -= AHRS->_beta * s1;
        qDot3 -= AHRS->_beta * s2;
        qDot4 -= AHRS->_beta * s3;
    }

    /* Integrate rate of change of quaternion to yield quaternion */
    AHRS->_q0 += qDot1 * AHRS->_sampleRate;
    AHRS->_q1 += qDot2 * AHRS->_sampleRate;
    AHRS->_q2 += qDot3 * AHRS->_sampleRate;
    AHRS->_q3 += qDot4 * AHRS->_sampleRate;

    /* Normalise quaternion */
    recipNorm = oneOverSqrt(AHRS->_q0 * AHRS->_q0 + AHRS->_q1 * AHRS->_q1 + AHRS->_q2 * AHRS->_q2 + AHRS->_q3 * AHRS->_q3);
    AHRS->_q0 *= recipNorm;
    AHRS->_q1 *= recipNorm;
    AHRS->_q2 *= recipNorm;
    AHRS->_q3 *= recipNorm;
    
    /* Calculate new angles */
    calculateAngles(AHRS);
}
