#ifndef _MPU_H_
#define _MPU_H_

#include "stm32f4xx_hal.h"

extern float MPU_get_temp(I2C_HandleTypeDef *hi2c);
extern void MPU_get_accel(I2C_HandleTypeDef *hi2c, int16_t *destination);
extern void MPU_get_gyro(I2C_HandleTypeDef *hi2c, int16_t * destination);
extern void MPU_Init(I2C_HandleTypeDef *hi2c);

#define MPU9250_ADDRESS_R 0xD1 // 11010001 = 0xD0 (AD0=0), 11010010 = 0xD2 (AD0=1)
#define MPU9250_ADDRESS_W 0xD1 

#define MPU_I2C_TIMEOUT 50 // in ms

#define REG_PWR_MGMT_1 0x6B
#define FLG_PWR_MGMT_1_DEFAULT 0x0
#define FLG_PWR_MGMT_1_H_RESET 0x80
#define FLG_PWR_MGMT_1_SLEEP 0x40
#define FLG_PWR_MGMT_1_CLKSEL_INTERNAL_20MHz 0x0
#define FLG_PWR_MGMT_1_CLKSEL_PLL_AUTO 0x1

#define REG_WHOAMI 0x75

/* 
Divides the internal sample rate (see register CONFIG) to generate the
sample rate that controls sensor data output rate, FIFO sample rate.
NOTE: This register is only effective when Fchoice = 2’b11 (fchoice_b
register bits are 2’b00), and (0 < dlpf_cfg < 7), such that the average filter’s
output is selected (see chart below).
This is the update rate of sensor register.
SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
*/
#define REG_SMPLRT_DIV 0x19
#define FLG_SMPLRT_DIV 0x04;


#define REG_CONFIG 0x1a
#define FLG_CONFIG_EXT_SYNC_SET_DISABLED 0x0
#define FLG_CONFIG_DLPF_CFG_41HZ 0x3

#define REG_GYRO_CFG 0x1b
#define FLG_GYRO_CFG_GYRO_FS_250DPS 0x0
#define MASK_GYRO_CFG_SELFTEST ((0x1 << 5) | (0x1 << 6) | (0x1 << 7)) // bits 7-5
#define MASK_GYRO_CFG_FCHOICE (0x1 << 1 | 0x1 )
#define MASK_GYRO_CFG_FS ((0x1 << 4) | (0x1 << 3))

#define REG_ACCEL_CFG 0x1c
#define MASK_ACCEL_CFG_SELFTEST ((0x1 << 5) | (0x1 << 6) | (0x1 << 7)) // bits 7-5
#define MASK_ACCEL_CFG_DLPFCFG ((0x1 << 1) | 0x1 )
#define MASK_ACCEL_CFG_FS ((0x1 << 4) | (0x1 << 3))
#define FLG_ACCEL_CFG_FS_2G ((0x0 << 3) | (0x0 << 4))

#define REG_ACCEL_CFG2 0x1d
#define MASK_GYRO_CFG2_FCHOICE (0x1 << 3)
#define MASK_ACCEL_CFG_DLPFCFG ((0x1 << 1) | 0x1 )
#define FLG_ACCEL_CFG_DLPFCFG 0x3


// Registers 59 to 64 â€“ Accelerometer Measurements 
// Name: ACCEL_XOUT_H 
// Serial IF: SyncR 
// Reset value: 0x00 (if sensor disabled) 
// ..
// Name: ACCEL_ZOUT_L 
#define REG_ACCEL_XOUT_H 0x3b //59

// Registers 65 and 66 â€“ Temperature Measurement
// Name: TEMP_OUT_H 
// Serial IF: SyncR 
// Reset value: 0x00 (if sensor disabled) 
#define REG_TEMP_OUT_H 0x41 // 65

// Registers 67 to 72 â€“ Gyroscope Measurements 
// Name: GYRO_XOUT_H 
// ...
// Name: GYRO_ZOUT_L 
// GYRO_ZOUT =  Gyro_Sensitivity * Z_angular_rate 
// -> Nominal : FS_SEL = 0 
// -> Conditions : Gyro_Sensitivity = 131 LSB/(Âº/s) 
#define REG_GYRO_XOUT_H 0x43 // 67

#endif // _MPU_H_
