#ifndef _MPU_H_
#define _MPU_H_

#include "stm32f4xx_hal.h"

extern float MPU_get_temp(I2C_HandleTypeDef *hi2c);
extern void MPU_get_accel(I2C_HandleTypeDef *hi2c, int16_t *destination);
extern void MPU_get_gyro(I2C_HandleTypeDef *hi2c, int16_t * destination);
extern HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c);
extern HAL_StatusTypeDef MPU_Init_MAGN(I2C_HandleTypeDef *hi2c);
extern int16_t MPU_GetFifoCnt( I2C_HandleTypeDef *hi2c );
extern HAL_StatusTypeDef MPU_GetFifoFrameData( I2C_HandleTypeDef *hi2c );
extern HAL_StatusTypeDef MPU_ResetFifo( I2C_HandleTypeDef *hi2c );

#define MPU_I2C_TIMEOUT 50 // in ms

#define MPU9250_ADDRESS_R 0xD1 // 11010001 = 0xD0 (AD0=0), 11010010 = 0xD2 (AD0=1)
#define MPU9250_ADDRESS_W MPU9250_ADDRESS_R 

#define AK8963_ADDRESS_W 0x0C
#define AK8963_ADDRESS_R AK8963_ADDRESS_W

#define MPU9250_FIFO_SIZE 512
// See register 35
// Contains: Temp (2) + 
#define MPU9250_FIFO_FRAME_SIZE (6+2+6+6) // ACCEL(3*2), TEMP(2), GYRO(3*2), SLAVE DATA (3*2?)

//  Register 25 – Sample Rate Divider 
// -------------------------------------------------
// SMPLRT_DIV 
#define MPU9250_REG25_ADDR 25
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
#define MPU9250_REG26_ADDR 26

// FIFO_MODE [6]
// When set to ‘1’, when the fifo is full, additional writes will not be written to fifo.  
// When set to ‘0’, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data. 
#define MPU9250_REG26_FIFO_NOT_OVERWRITE (0x1 << 6)
#define MPU9250_REG26_FIFO_OVERWRITE (0x0 << 6)

// DLPF_CFG [2:0]
// For the DLPF to be used, fchoice[1:0] must be set to 2’b11, fchoice_b[1:0] is  2’b00. See table 3 below. 
// GYRO: bandwidth = 41 hz
// TEMP: bandwidth = 42 hz 
#define MPU9250_REG26_DLPF_CFG 0x3
#define MPU9250_REG26_DEFAULT (MPU9250_REG26_FIFO_NOT_OVERWRITE | MPU9250_REG26_DLPF_CFG)


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
//#define MPU9250_REG35_DEFAULT (MPU9250_REG35_TEMP_OUT|MPU9250_REG35_GYRO_XOUT|MPU9250_REG35_GYRO_YOUT|MPU9250_REG35_GYRO_ZOUT|MPU9250_REG35_ACCEL)
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



/*
 
BIT  NAME  FUNCTION 
[7:0]  D[7:0] 
Low byte of the Z-Axis gyroscope output 
GYRO_ZOUT =  Gyro_Sensitivity * Z_angular_rate 
Nominal 
Conditions 
FS_SEL = 0 
Gyro_Sensitivity = 131 LSB/(º/s) 
 
 
 
4.25  Registers 73 to 96 – External Sensor Data 
 
EXT_SENS_DATA_00 – 23 
Serial IF: SyncR 
Reset value: 0x00 
 
24 registers with the same description as below: 
BIT  NAME  FUNCTION 
[7:0]  D[7:0] 
Sensor data read from external I2C devices via the I2C 
master interface.  The data stored is controlled by the 
I2C_SLV(0-4)_ADDR, I2C_SLV(0-4)_REG, and 
I2C_SLV(0-4)_CTRL registers 
Description: 
These registers store data read from external sensors by the Slave 0, 1, 2, and 3 on the auxiliary I
2
C 
interface. Data read by Slave 4 is stored in I2C_SLV4_DI (Register 53).  
External sensor data is written to these registers at the Sample Rate as defined in Register 25. This 
access rate can be reduced by using the Slave Delay Enable registers (Register 103). 
Data  is  placed  in  these  external  sensor  data  registers  according  to  I2C_SLV0_CTRL, 
I2C_SLV1_CTRL,  I2C_SLV2_CTRL,  and  I2C_SLV3_CTRL  (Registers  39,  42,  45,  and  48).  When 
more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the 
slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in Register 
52 and 103). During each sample cycle, slave reads are performed in order of Slave number. If all 
slaves are enabled with more than zero bytes to be read, the order will be Slave 0, followed by Slave 
1, Slave 2, and Slave 3.  
Each enabled slave will have EXT_SENS_DATA registers associated with it by number of bytes read 
(I2C_SLVx_LEN)  in  order  of  slave  number,  starting  from  EXT_SENS_DATA_00.  Note  that  this 
means enabling or disabling a slave may change the higher numbered slaves’ associated registers. 
Furthermore,  if  fewer  total  bytes  are  being  read  from  the  external  sensors  as  a  result  of  such  a 
change, then the data remaining in the registers which no longer have an associated slave device 
(i.e. high numbered registers) will remain in these previously allocated registers unless reset. 
If  the  sum  of  the  read  lengths  of  all  SLVx  transactions  exceed  the  number  of  available 
EXT_SENS_DATA  registers,  the  excess  bytes  will  be  dropped.  There  are  24  EXT_SENS_DATA 
registers and hence the total read lengths between all the slaves cannot be greater than 24 or some 
bytes will be lost. 
Note:  Slave  4’s  behavior  is  distinct  from  that  of  Slaves  0-3.  For  further  information  regarding  the 
characteristics of Slave 4, please refer to Registers 49 to 53.  

Example: 
Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and I2C_SLV0_LEN = 
4) while Slave 1 is enabled with 2 bytes to be read, (I2C_SLV1_EN=1 and I2C_SLV1_LEN = 2). In 
such  a  situation,  EXT_SENS_DATA  _00  through  _03  will  be  associated  with  Slave  0,  while 
EXT_SENS_DATA _04 and 05 will be associated with Slave 1.  
If Slave 2 is enabled as well, registers starting from EXT_SENS_DATA_06 will be allocated to Slave 
2. 
If  Slave  2  is  disabled  while  Slave  3  is  enabled  in  this  same  situation,  then  registers  starting  from 
EXT_SENS_DATA_06 will be allocated to Slave 3 instead.  

*/

// Registers 37 to 39 – I2C Slave 0 Control
// -------------------------------------------------
// I2C_SLV0_RNW [7] - 1 -  Transfer is a read;  0 - Transfer is a write 
// I2C_ID_0 [6:0] - Physical address of I2C slave 0 

// Register 37 - I2C_SLV0_ADDR
#define MPU9250_REG37_ADDR 37
#define MPU9250_REG37_I2C_SLV0_RNW_READ (0x1 << 7)  // READ MODE
#define MPU9250_REG37_I2C_ID_0 (AK8963_ADDRESS_R)
#define MPU9250_REG37_DEFAULT (MPU9250_REG37_I2C_ID_0 | MPU9250_REG37_I2C_SLV0_RNW_READ)

// Register 38 - I2C_SLV0_REG  
#define MPU9250_REG38_ADDR 38
#define MPU9250_REG38_DEFAULT  (AK8963_REG0_ADDR)

// Register 39 - I2C_SLV0_CTRL 
#define MPU9250_REG39_ADDR 39
// 1 – Enable reading data from this slave at the sample rate and storing 
// data at the first available EXT_SENS_DATA register, which is always 
// EXT_SENS_DATA_00 for I2C slave 0. 
#define MPU9250_REG39_I2C_SLV0_EN (0x1 << 7)
// I2C_SLV0_LENG[3:0] 
#define MPU9250_REG39_I2C_SLV0_LENG 1 // bytes to read
#define MPU9250_REG39_DEFAULT (MPU9250_REG39_I2C_SLV0_EN | MPU9250_REG39_I2C_SLV0_LENG)


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
#define MPU9250_REG55_LATCH_INT_EN (0x1 << 5)
// INT_ANYRD_2CLEAR [4]
// 1 – Interrupt status is cleared if any read operation is performed. 
#define MPU9250_REG55_INT_ANYRD_2CLEAR (0x1 << 4)
#define MPU9250_REG55_DEFAULT (MPU9250_REG55_BYPASS_EN | MPU9250_REG55_LATCH_INT_EN)
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
#define MPU9250_REG73_ADDR 73
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
#define MPU9250_REG107_DEFAULT 0x0

// CLKSEL: 2 bits
// 1 - Auto selects the best available clock source – PLL if ready, else  use the Internal oscillator  
#define MPU9250_REG107_CLKSEL_DEFAULT 0x1


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
#define AK8963_REG0_ADDR 0x0
#define AK8963_REG0_WAI_ID 0x48


#endif // _MPU_H_
