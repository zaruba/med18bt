#ifndef _MPU_H_
#define _MPU_H_

#include "stm32f4xx_hal.h"

extern HAL_StatusTypeDef MPU_get_temp(I2C_HandleTypeDef *hi2c, float *temp);
extern HAL_StatusTypeDef MPU_get_accel(I2C_HandleTypeDef *hi2c, float *destination);
extern HAL_StatusTypeDef MPU_get_gyro(I2C_HandleTypeDef *hi2c, float *destination);
extern HAL_StatusTypeDef MPU_get_magn(I2C_HandleTypeDef *hi2c, float *destination);

extern HAL_StatusTypeDef MPU_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data, uint8_t bytes_to_read);
extern HAL_StatusTypeDef MPU_Write(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data);

extern HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c, uint8_t hard_reset_required);
extern HAL_StatusTypeDef MPU_Init_MAGN(I2C_HandleTypeDef *hi2c);
extern int16_t MPU_GetFifoCnt( I2C_HandleTypeDef *hi2c );

extern HAL_StatusTypeDef MPU_GetData( I2C_HandleTypeDef *hi2c, float *accel_data, float *gyro_data, float *magn_data, float *temp );

extern HAL_StatusTypeDef MPU_GetFifoFrameData( I2C_HandleTypeDef *hi2c, float *accel_data, float *gyro_data, float *magn_data, float *temp );
extern HAL_StatusTypeDef MPU_ResetFifo( I2C_HandleTypeDef *hi2c );

extern HAL_StatusTypeDef AK8963_SL0_Setup(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t bytes_to_read);
extern HAL_StatusTypeDef AK8963_SL0_Read(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t *data, uint8_t bytes_to_read);
extern HAL_StatusTypeDef AK8963_SL0_Write(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t data);

extern HAL_StatusTypeDef AK8963_Read(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t *data, uint8_t bytes_to_read);
extern HAL_StatusTypeDef AK8963_Write(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t data);


#define MPU_I2C_TIMEOUT 50 // in ms

#define MPU9250_ADDRESS_R 0xD1 // 11010001 = 0xD0 (AD0=0), 11010010 = 0xD2 (AD0=1)
#define MPU9250_ADDRESS_W MPU9250_ADDRESS_R 

#define AK8963_ADDRESS_R (0x0C)
#define AK8963_ADDRESS_W AK8963_ADDRESS_R

#define MPU9250_FIFO_SIZE 512
// See register 35
// Contains: Temp (2) + 
#define MPU9250_FIFO_FRAME_SIZE (6+2+6+AK8963_FRAME_SZ) // ACCEL(3*2), TEMP(2), GYRO(3*2), SLAVE DATA (ST1 + 3*2 + ST2)

//  Register 25 – Sample Rate Divider 
// -------------------------------------------------
// SMPLRT_DIV 
#define MPU9250_REG25_SMPLRT_DIV_ADDR 25
// Divides  the  internal  sample  rate  (see  register  CONFIG)  to  generate  the 
// sample  rate  that  controls  sensor  data  output  rate,  FIFO  sample  rate.  
// NOTE:  This  register  is  only  effective  when  Fchoice  =  2’b11  (fchoice_b 
// register bits are 2’b00), and (0 < dlpf_cfg < 7), such that the average filter’s 
// output is selected (see chart below). 
// This is the update rate of sensor register. 
// SAMPLE_RATE=  Internal_Sample_Rate / (1 + SMPLRT_DIV) 
// Data should be sampled at or above sample rate; SMPLRT_DIV is only used for1kHz internal sampling.
#define MPU9250_REG25_SMPLRT_DIV_DEFAULT 0x4

// Register 26 – Configuration 
// -------------------------------------------------
// CONFIG
#define MPU9250_REG26_CONFIG_ADDR 26

// FIFO_MODE [6]
// When set to ‘1’, when the fifo is full, additional writes will not be written to fifo.  
// When set to ‘0’, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data. 
#define MPU9250_REG26_FIFO_NOT_OVERWRITE (0x1 << 6)
#define MPU9250_REG26_FIFO_OVERWRITE (0x0 << 6)

// DLPF_CFG [2:0]
// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
// be higher than 1 / 0.0059 = 170 Hz
// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz 
#define MPU9250_REG26_DLPF_CFG 0x3

// Register 27 – Gyroscope Configuration
// -------------------------------------------------
#define MPU9250_REG27_GYRO_ADDR 27
#define MPU9250_REG27_GYRO_SELFTEST_MASK ((0x1<<7) | (0x1<<6) | (0x1<<5)) // bits 7-5
#define MPU9250_REG27_GYRO_FS_MASK ((0x1<<4) | (0x1<<3))
#define MPU9250_REG27_GYRO_FCHOICE_MASK (0x1 << 1 | 0x1 )
#define MPU9250_REG27_GYRO_FS_250DPS (0x0<<3)
#define MPU9250_GYRO_250DPS_RESOLUTION (250.0f/32768.0f) 

// Register 28 – Accelerometer Configuration
// -------------------------------------------------
#define MPU9250_REG28_ACCEL_ADDR 28
#define MPU9250_REG28_ACCEL_SELFTEST_MASK ((0x1<<7) | (0x1<<6) | (0x1<<5)) // bits 7-5
#define MPU9250_REG28_ACCEL_CFG_FS_MASK ((0x1<<4) | (0x1<<3))
#define MPU9250_REG28_ACCEL_DLPFCFG_MASK ((0x1 << 1) | 0x1 )
#define MPU9250_REG28_ACCEL_FS_2G ((0x0<<4) | (0x0<<3))
#define MPU9250_ACCEL_2G_RESOLUTION (2.0f/32768.0f) 

// Register 29 – Accelerometer Configuration 2
// -------------------------------------------------
#define MPU9250_REG29_ACCEL2_ADDR 29

#define MPU9250_REG29_ACCEL2_FCHOICE_MASK (0x1 << 3)
#define MPU9250_REG29_ACCEL2_DLPFCFG_MASK ((0x1 << 1) | 0x1 )
#define MPU9250_REG29_ACCEL2_DLPFCFG 0x3


// Register 35 – FIFO Enable 
// -------------------------------------------------
#define MPU9250_REG35_ADDR 35
// TEMP_OUT [7]
// 1 – Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate; 
// If enabled, buffering of data occurs even if data path is in standby. 
#define MPU9250_REG35_TEMP_OUT (0x1 << 7)
// GYRO_XOUT [6]
#define MPU9250_REG35_GYRO_XOUT (0x1 << 6)
// GYRO_YOUT [5]
#define MPU9250_REG35_GYRO_YOUT (0x1 << 5)
// GYRO_ZOUT [4]
#define MPU9250_REG35_GYRO_ZOUT (0x1 << 4)
// ACCEL [3]
#define MPU9250_REG35_ACCEL (0x1 << 3)
// SLV_0 [0] 
// 1 – write EXT_SENS_DATA registers associated to SLV_0 (as determined by 
// I2C_SLV0_CTRL)  to the FIFO at the sample rate; 
#define MPU9250_REG35_SLV_0 (0x1)
#define MPU9250_REG35_DEFAULT (MPU9250_REG35_TEMP_OUT|MPU9250_REG35_GYRO_XOUT|MPU9250_REG35_GYRO_YOUT|MPU9250_REG35_GYRO_ZOUT|MPU9250_REG35_ACCEL|MPU9250_REG35_SLV_0)

// Register 36 – I2C Master Control 
// -------------------------------------------------
// I2C_MST_CTRL
#define MPU9250_REG36_ADDR 36
// MULT_MST_EN [7]
// Enables multi-master capability.  When disabled, clocking to the I2C_MST_IF 
// can be disabled when not in use and the logic to detect lost arbitration is disabled.
#define MPU9250_REG36_MULT_MST_EN (0x1 << 7)
// WAIT_FOR_ES [6]
// Delays the data ready interrupt until external sensor data is loaded.  If 
// I2C_MST_IF is disabled, the interrupt will still occur.  
#define MPU9250_REG36_WAIT_FOR_ES (0x1 << 6)
// SLV_3_FIFO_EN [5]
// 1 – write EXT_SENS_DATA registers associated to SLV_3 (as determined by 
// I2C_SLV0_CTRL and I2C_SLV1_CTRL and I2C_SLV2_CTRL)  to the FIFO at  the sample rate; 
// 0 – function is disabled
#define MPU9250_REG36_SLV_3_FIFO_EN (0x1 << 5)
// I2C_MST_P_NSR [4]
// This bit controls the I2C Master’s transition from one slave read to the next 
// slave read.  If 0, there is a restart between reads.  If 1, there is a stop between reads.
#define MPU9250_REG36_I2C_MST_P_NSR (0x1 << 4)
// I2C_MST_CLK [3:0]
// I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU-9250 internal 8MHz clock. 
// It sets the I2C master clock speed according to the following table:  <skipped>
#define MPU9250_REG36_I2C_MST_CLK_400KHZ 13 // I2C clock speed is 400 kHz 

#define MPU9250_REG36_DEFAULT (MPU9250_REG36_MULT_MST_EN | MPU9250_REG36_WAIT_FOR_ES | MPU9250_REG36_I2C_MST_CLK_400KHZ)

// Registers 37 to 39 – I2C Slave 0 Control
// -------------------------------------------------
// I2C_SLV0_RNW [7] - 1 -  Transfer is a read;  0 - Transfer is a write 
// I2C_ID_0 [6:0] - Physical address of I2C slave 0 

// Register 37 - I2C_SLV0_ADDR
#define MPU9250_SLAVE0_BASE 37
#define MPU9250_REG37_I2C_SLV0_RNW_READ (0x1 << 7)  // READ MODE
#define MPU9250_REG37_I2C_ID_0 (AK8963_ADDRESS_R)
#define MPU9250_REG37_AK8963_READ (MPU9250_REG37_I2C_ID_0 | MPU9250_REG37_I2C_SLV0_RNW_READ)
#define MPU9250_REG37_AK8963_WRITE (MPU9250_REG37_I2C_ID_0)

// Register 38 - I2C_SLV0_REG  
#define MPU9250_REG38_WAI_REG AK8963_REG0_ADDR  // WHO AM I @ AK8963

// Register 39 - I2C_SLV0_CTRL 
#define MPU9250_REG39_ADDR 39
// 1 – Enable reading data from this slave at the sample rate and storing 
// data at the first available EXT_SENS_DATA register, which is always 
// EXT_SENS_DATA_00 for I2C slave 0. 
#define MPU9250_REG39_I2C_SLV0_EN (0x1 << 7)
// I2C_SLV0_LENG[3:0] 
#define MPU9250_REG39_ENABLE_1b (MPU9250_REG39_I2C_SLV0_EN | 1)
#define MPU9250_REG39_ENABLE_7b (MPU9250_REG39_I2C_SLV0_EN | 7)
#define MPU9250_REG39_DISABLE 0x0 // set 7 bit to zero

// Registers 40 to 42 – I2C Slave 1 Control 
// -------------------------------------------------

// Register 40 - I2C_SLV1_CTRL 
#define MPU9250_REG40_ADDR 40
#define MPU9250_REG40_I2C_SLV1_RNW_READ (0x1 << 7)  // READ MODE
#define MPU9250_REG40_I2C_ID_1 (AK8963_ADDRESS_R)
#define MPU9250_REG40_AK8963_READ (MPU9250_REG40_I2C_ID_1 | MPU9250_REG40_I2C_SLV1_RNW_READ)
#define MPU9250_REG40_AK8963_WRITE (MPU9250_REG40_I2C_ID_1)

// Register 41 - I2C_SLV1_REG  
#define MPU9250_REG41_ADDR 41
#define MPU9250_REG41_DEFAULT 0x3 // X-Low, X-High, Y-Low etc

// Register 42 - I2C_SLV1_CTRL 
#define MPU9250_REG42_ADDR 42
// 1 – Enable reading data from this slave at the sample rate and storing 
// data at the first available EXT_SENS_DATA register, which is always 
// EXT_SENS_DATA_00 for I2C slave 0. 
#define MPU9250_REG42_I2C_SLV1_EN (0x1 << 7)
// I2C_SLV0_LENG[3:0] 
#define MPU9250_REG42_I2C_SLV1_LENG 6 // bytes to read

#define MPU9250_REG42_DISABLE 0x0 // set 7 bit to zero
#define MPU9250_REG42_ENABLE (MPU9250_REG42_I2C_SLV1_EN | MPU9250_REG42_I2C_SLV1_LENG)
#define MPU9250_REG42_DISABLE 0x0


// Registers 43 to 45 – I2C Slave 2 Control 
// -------------------------------------------------

// Register 45 - I2C_SLV2_CTRL 
#define MPU9250_REG45_ADDR 45
#define MPU9250_REG45_DISABLE 0x0


// Registers 46 to 48 – I2C Slave 3 Control 
// -------------------------------------------------
#define MPU9250_REG46_ADDR 46

// Register 46 - I2C_SLV3_ADDR 
#define MPU9250_REG46_I2C_SLV3_RNW (0x1 << 7) // read mode
#define MPU9250_REG46_I2C_ID_3 (AK8963_ADDRESS_R)
#define MPU9250_REG46_DEFAULT (MPU9250_REG46_I2C_SLV3_RNW | MPU9250_REG46_I2C_ID_3)

// Register 47 - I2C_SLV3_REG [7:0]
// I2C slave 3 register address from where to begin data transfer 
#define MPU9250_REG47_ADDR 47
#define MPU9250_REG47_DEFAULT  (AK8963_REG3_ADDR)  // WHO AM I @ AK8963

// Register 48 - I2C_SLV3_CTRL 
#define MPU9250_REG48_ADDR 48
// 1 – Enable reading data from this slave at the sample rate and storing 
// data at the first available EXT_SENS_DATA register, which is always 
// EXT_SENS_DATA_00 for I2C slave 0. 
#define MPU9250_REG48_I2C_SLV3_EN (0x1 << 7)
// I2C_SLV0_LENG[3:0] 
#define MPU9250_REG48_I2C_SLV3_LENG 6 // 6 bytes to read: X, Y, Z
#define MPU9250_REG48_ENABLE (MPU9250_REG48_I2C_SLV3_EN | MPU9250_REG48_I2C_SLV3_LENG)
#define MPU9250_REG48_DISABLE 0x0 // set 7 bit to zero


//  Registers 49 to 53 – I2C Slave 4 Control 
// -------------------------------------------------
// Register 49 - I2C_SLV4_ADDR
#define MPU9250_REG49_ADDR 49
#define MPU9250_REG49_I2C_SLV4_RNW_READ (0x1 << 7)
#define MPU9250_REG49_I2C_ID_4_DEFAULT (AK8963_ADDRESS_R | MPU9250_REG49_I2C_SLV4_RNW_READ) // READ MODE

// Register 50 - I2C_SLV4_REG  
#define MPU9250_REG50_ADDR 50
#define MPU9250_REG50_I2C_SLV4_REG_DEFAULT (AK8963_REG0_ADDR)

// Register 52 - I2C_SLV4_CTRL
#define MPU9250_REG52_ADDR 52
// 1 – Enable data transfer with this slave at the sample rate. If read 
// command, store data in I2C_SLV4_DI register, if write command, write data 
// stored in I2C_SLV4_DO register.  Bit is cleared when a single transfer is 
// complete.  Be sure to write I2C_SLV4_DO first 
#define MPU9250_REG52_I2C_SLV4_EN_ENABLE (0x1 << 7)
#define MPU9250_REG52_I2C_SLV4_CTRL_DEFAULT (MPU9250_REG52_I2C_SLV4_EN_ENABLE)
	
// Register 53 - I2C_SLV4_DI 
#define MPU9250_REG53_ADDR 53

// Register 55 – INT Pin / Bypass Enable Configuration 
// -------------------------------------------------
// INT_PIN_CFG
#define MPU9250_REG55_ADDR 55

// BYPASS_EN [1]
// When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go 
// into ‘bypass mode’ when the i2c master interface is disabled.  The pins 
// will float high due to the internal pull-up if not enabled and the i2c master 
// interface is disabled.
#define MPU9250_REG55_BYPASS_EN (0x2)
// LATCH_INT_EN [5]
// 1 – INT pin level held until interrupt status is cleared. 
#define MPU9250_REG55_LATCH_INT_EN (0x1<<5)
// INT_ANYRD_2CLEAR [4]
// 1 – Interrupt status is cleared if any read operation is performed. 
#define MPU9250_REG55_INT_ANYRD_2CLEAR (0x1<<4)
#define MPU9250_REG55_ENABLE_BYPASS (MPU9250_REG55_BYPASS_EN | MPU9250_REG55_LATCH_INT_EN)
//#define MPU9250_REG55_DEFAULT (MPU9250_REG55_BYPASS_EN)

// Register 56 – Interrupt Enable 
// -------------------------------------------------
// INT_ENABLE
#define MPU9250_REG56_ADDR 56

// RAW_RDY_EN
// 1 – Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. 
// The timing of the interrupt can vary depending on the setting in register 36 
// I2C_MST_CTRL, bit [6] WAIT_FOR_ES. 
#define MPU9250_REG56_RAW_RDY_EN_ENABLE (0x1) // bit 0
#define MPU9250_REG56_DEFAULT (MPU9250_REG56_RAW_RDY_EN_ENABLE)


// Registers 59 to 64 – Accelerometer Measurements 
// Name: ACCEL_XOUT_H 
// Serial IF: SyncR 
// Reset value: 0x00 (if sensor disabled) 
// ..
// Name: ACCEL_ZOUT_L 
#define REG_ACCEL_XOUT_H 0x3b //59

// Registers 65 and 66 – Temperature Measurement
// Name: TEMP_OUT_H 
// Serial IF: SyncR 
// Reset value: 0x00 (if sensor disabled) 
#define REG_TEMP_OUT_H 0x41 // 65

// Registers 67 to 72 – Gyroscope Measurements 
// Name: GYRO_XOUT_H 
// ...
// Name: GYRO_ZOUT_L 
// GYRO_ZOUT =  Gyro_Sensitivity * Z_angular_rate 
// -> Nominal : FS_SEL = 0 
// -> Conditions : Gyro_Sensitivity = 131 LSB/(º/s) 
#define REG_GYRO_XOUT_H 0x43 // 67

// Register 73
// -------------------------------------------------
// Registers 73 to 96 – External Sensor Data
#define MPU9250_SLAVE0_DATA_ADDR 73
// Sensor data read from external I2C devices via the I2C  master interface.  The data stored is controlled by the 
// I2C_SLV(0-4)_ADDR, I2C_SLV(0-4)_REG, and I2C_SLV(0-4)_CTRL registers 


// Register 106 – User Control 
// -------------------------------------------------
#define MPU9250_REG106_ADDR 106
// FIFO_EN [6]
// 1 – Enable FIFO operation mode. 
#define MPU9250_REG106_FIFO_EN (0x1 << 6)
// USER_CTRL [5]
// 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK. 
// 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically driven by pins SDA/SDI and SCL/ SCLK. 
#define MPU9250_REG106_I2C_MST_EN (0x1 << 5)
// [2]  FIFO_RST 
// 1 – Reset FIFO module. Reset is asynchronous.  This bit auto clears after  one clock cycle. 
#define MPU9250_REG106_FIFO_RST (0x1 << 2)


// Register 107 – Power Management 1 
// -------------------------------------------------
// PWR_MGMT_1 
#define MPU9250_REG107_ADDR 107
// H_RESET [7]
// 1 – Reset the internal registers and restores the default settings.  Write a 1 to set the reset, the bit will auto clear. 
#define MPU9250_REG107_H_RESET (0x1 << 7) 
// CLKSEL: 2 bits
#define MPU9250_REG107_CLKSEL_INTERNAL 0x0
// 1 - Auto selects the best available clock source – PLL if ready, else  use the Internal oscillator  
#define MPU9250_REG107_CLKSEL_PLL 0x1

// -------------------------------------------------
#define MPU9250_REG116_ADDR 116

// Register 117 – Who Am I
// -------------------------------------------------
// WHOAMI

#define MPU9250_REG117_ADDR 117
// Register to indicate to user which device is being accessed. 
// This register is used to verify the identity of the device. 
// The contents of WHO_AM_I is an 8-bit device ID. 
// The default value of the register is 0x71.  
#define MPU9250_REG117_WAI_ID 0x71

/////////////////////////////////////////////////////////////////////////////////////////////
// MAGN
/////////////////////////////////////////////////////////////////////////////////////////////

// Name					WIA
// Address			0x00
// READ/WRITE		READ
// Description 	Device ID (Device ID of AKM. It is described in one byte and fixed value: 0x48)
// Bit width 		8
#define AK8963_WAI_ADDR 0x0
#define AK8963_HXL_ADDR 0x2 // 8 bytes: ST1, HXL, HXH, XYL, XYH, XZL, XZH, ST2 - see MPU9250_FIFO_FRAME_SIZE
#define AK8963_FRAME_SZ 8
#define AK8963_REG0_WAI_ID 0x48

#define AK8963_MAG_BIAS_X 0 // User environmental x-axis correction in milliGauss, should be automatically calculated
#define AK8963_MAG_BIAS_Y 0 // User environmental x-axis correction in milliGauss
#define AK8963_MAG_BIAS_Z 0 // User environmental x-axis correction in milliGauss
		
#define AK8963_SCALE_FACTOR (4912.0f / 32760.0f) // 0.149..f OR ((float)0.15f) 
	
#endif // _MPU_H_
