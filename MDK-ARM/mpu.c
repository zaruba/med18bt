#include "mpu.h"

#include "stm32f4xx_hal.h"

// TEMP_degC   = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC 
// Where Temp_degC is the temperature in degrees C measured by the temperature sensor.  
// TEMP_OUT is the actual output of the temperature sensor. 

#define ROOM_TEMP_OFFSET 21 // in C

float MPU_get_temp(I2C_HandleTypeDef *hi2c) {
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	float res;

	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_TEMP_OUT_H, 1, rawData, 2, MPU_I2C_TIMEOUT);  // Read the two raw data registers sequentially into data array
	res = (((uint16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
	
  res = (float) ( res / 100.0f );
	
	return res;
}

void MPU_get_accel(I2C_HandleTypeDef *hi2c, int16_t *destination) {
	uint8_t rawData[6];

	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_ACCEL_XOUT_H, 1, rawData, 6, MPU_I2C_TIMEOUT);
	
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU_get_gyro(I2C_HandleTypeDef *hi2c, int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_GYRO_XOUT_H, 1, rawData, 6, MPU_I2C_TIMEOUT);    // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

}

void MPU_Init(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef res;
	uint8_t t;
	
	// get WHOAMI ID
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_WHOAMI, 1, &t, 8, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: Can't get MPU ID (I2C)\r\n");
		return;
	}
	
	if ( (t&0xff) != 0x71) {
		printf("WARNING: device has invalid device ID=%x (I2C)\r\n", (t&0xff));
	}
		
	// Clear sleep mode bit (6), reset and enable all sensors 
	// t = (FLG_PWR_MGMT_1_DEFAULT | FLG_PWR_MGMT_1_CLKSEL_PLL_AUTO);
	t = FLG_PWR_MGMT_1_DEFAULT; // = FLG_PWR_MGMT_1_CLKSEL_INTERNAL_20MHz
	res = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, REG_PWR_MGMT_1, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: Can't init MPU (I2C)\r\n");
		return;
	}
	printf("DEBUG: PWR_MGMT_1: 0x%x\r\n", t);
	
	HAL_Delay(50);

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	t = FLG_CONFIG_EXT_SYNC_SET_DISABLED | FLG_CONFIG_DLPF_CFG_41HZ;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, FLG_CONFIG_DLPF_CFG_41HZ, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: Can't init MPU: DLPF_CFG (I2C)\r\n");
		return;
	}
	printf("DEBUG: DLPF_CFG: 0x%x\r\n", t);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	t = FLG_SMPLRT_DIV;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, REG_SMPLRT_DIV, 1, &t, 1, MPU_I2C_TIMEOUT);   // Use a 200 Hz rate; the same rate set in CONFIG above
	printf("DEBUG: SMPLRT_DIV: 0x%x\r\n", t);
   
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_GYRO_CFG, 1, &t, 6, MPU_I2C_TIMEOUT);    // get current GYRO_CONFIG register value
	
  t &= ~MASK_GYRO_CFG_SELFTEST; // Clear self-test bits [7:5]
	t &= ~MASK_GYRO_CFG_FS; // Clear AFS bits [4:3]
	t &= ~MASK_GYRO_CFG_FCHOICE; // Clear Fchoice bits [1:0]
	t |= FLG_GYRO_CFG_GYRO_FS_250DPS; // Set full scale range for the gyro - 0=+250dps
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, REG_GYRO_CFG, 1, &t, 1, MPU_I2C_TIMEOUT);    // Write new GYRO_CONFIG value to register
	printf("DEBUG: GYRO_CFG: 0x%x\r\n", t);
 
  // Set accelerometer full-scale range configuration
  HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_ACCEL_CFG, 1, &t, 6, MPU_I2C_TIMEOUT);   // get current ACCEL_CONFIG register value
	
  t &= ~MASK_ACCEL_CFG_SELFTEST; // Clear self-test bits [7:5]
  t &= ~MASK_ACCEL_CFG_FS;  // Clear AFS bits [4:3]
  t |= FLG_ACCEL_CFG_FS_2G; // Set full scale range for the accelerometer  - 0=2g
  HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, REG_ACCEL_CFG, 1, &t, 1, MPU_I2C_TIMEOUT);   // Write new ACCEL_CONFIG register value
	printf("DEBUG: ACCEL_CFG: 0x%x\r\n", t);
   
  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, REG_ACCEL_CFG2, 1, &t, 6, MPU_I2C_TIMEOUT);   // get current ACCEL_CONFIG2 register value

	t &= ~MASK_GYRO_CFG2_FCHOICE; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) 
  t |= FLG_ACCEL_CFG_DLPFCFG;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, REG_ACCEL_CFG2, 1, &t, 1, MPU_I2C_TIMEOUT);   // Write new ACCEL_CONFIG2 register value
	printf("DEBUG: ACCEL_CFG2: 0x%x\r\n", t);
   
   // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
   // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

   // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   //R=0x22;
   //HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, INT_PIN_CFG, 1, &R, 1, 100);
  //R=0x01;
   //HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, INT_ENABLE, 1, &R, 1, 100);   // Enable data ready (bit 0) interrupt
  
//http://radiokot.ru/forum/viewtopic.php?f=2&t=136480
//setFullScaleGyroRange(MPU9250_GYRO_FULL_SCALE_250DPS);
//    setFullScaleAccelRange(MPU9250_FULL_SCALE_8G); 	
	MPU_Init_MAGN(hi2c);
	
	printf("DEBUG: MPU initliazed\r\n");
}

//

void MPU_Init_MAGN(I2C_HandleTypeDef *hi2c) {
  
	uint8_t R;
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here

	// First extract the factory calibration for each magnetometer axis
	R=0x00;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Power down magnetometer 
	HAL_Delay(10);
	
	R=0x0F;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Enter Fuse ROM access mode
	HAL_Delay(10);
	
	HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS_R, AK8963_ASAX, 1, rawData, 3, 100);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f; 
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
	R=0x00;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Power down magnetometer 
	HAL_Delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	R = 0 << 4 | 0x06;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Set magnetometer data resolution and sample ODR
	HAL_Delay(10);

	R=0x40;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_ASTC, 1, &R, 1, 100); // set self-test
	//   
}
//
void MPU_get_magn(I2C_HandleTypeDef *hi2c, int16_t * destination) {

	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t c;

	HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1, 100);
	
	if(c >= 0x01) { // wait for magnetometer data ready bit to be set
		
		HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7, 100);  // Read the six raw data and ST2 registers sequentially into data array
		c = rawData[6]; // End data read by reading ST2 register
		
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
			destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
		}
	}

}
