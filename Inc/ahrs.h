#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f4xx_hal.h"

#include "stdint.h"
#include "math.h"
    
/**
 * @defgroup AHRS_Macros
 * @brief    Library defines
 * @{
 */

#define AHRS_PI              3.141592653f            /*!< PI definition */
#define AHRS_RAD2DEG(x)      ((x) * 57.2957795f)     /*!< Radians to degrees converter */
#define AHRS_DEG2RAD(x)      ((x) * 0.0174532925f)   /*!< degrees to radians converter */

/**
 * @brief   Main working AHRS IMU structure
 */
typedef struct AHRS_t {
    float Roll;             /*!< Roll angle value. This parameter is in units of degrees */
    float Pitch;            /*!< Pitch angle value. This parameter is in units of degrees */
    float Yaw;              /*!< Yaw angle value. This parameter is in units of degrees */
    float Inclination;      /*!< Inclination in units of degrees */
    
    float _beta;
    float _q0, _q1, _q2, _q3;
    float _sampleRate;
} AHRS_t;

/**
 * @}
 */

/**
 * @defgroup AHRS_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * \brief  Initializes ARHS or IMU algorithm for motion calculations
 * \param  *AHRS: Pointer to \ref AHRS_t empty structure
 * \param  sampleRate: Sample rate frequency for updating data
 * \param  beta: Gain for calculations and speed to stabilize. A value less than 0.2 is a good value but it mostly depends on applicaiton.
 * \param  inclination: Magnetic inclination for position on earth in units of degrees. This value depends on GPS coordinates on earth.
 * \retval None
 */
void AHRS_Init(AHRS_t* AHRS, float sampleRate, float beta, float inclination);

/**
 * \brief  Updates AHRS algorithm with new values and calculates roll, pitch and yaw angles.
 * \param  *AHRS: Pointer to \ref AHRS_t empty structure
 * \param  ax, ay, az: Accelerometer data for X, Y and Z axis respectively. These parameters must be in units of gees.
 * \param  gx, gy, gz: Gyro data for X, Y and Z axis respectively. These parameters must be in units of rad/s. Use \ref AHRSIMU_DEG2RAD if your gyro outputs data in deg/s.
 * \param  mx, my, mz: Magnetic data for X, Y and Z axis respectively. These parameters must be in units of uT (micro Tesla).
 *            When all parameters are zero (magnetic sensor not used), \ref TM_AHRSIMU_UpdateIMU is automatically called.
 * \retval None
 */
void AHRS_UpdateAHRS(AHRS_t* AHRS, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

/**
 * \brief  Updates IMU algorithm with new values and calculates roll, pitch and yaw angles.
 * \param  *AHRS: Pointer to \ref AHRS_t empty structure
 * \param  ax, ay, az: Accelerometer data for X, Y and Z axis respectively. These parameters must be in units of gees.
 * \param  gx, gy, gz: Gyro data for X, Y and Z axis respectively. These parameters must be in units of rad/s. Use \ref AHRSIMU_DEG2RAD if your gyro outputs data in deg/s.
 * \retval None
 */
void AHRS_UpdateIMU(AHRS_t* AHRS, float ax, float ay, float az, float gx, float gy, float gz);

/**
 * \brief  Se	ts new gain value for processing
 * \param  *AHRS: Pointer to \ref AHRS_t empty structure
 * \param  beta: New beta value
 * \retval None
 */
#define AHRS_SetBeta(AHRS, beta)       ((AHRS)->_beta = (beta))


#endif
