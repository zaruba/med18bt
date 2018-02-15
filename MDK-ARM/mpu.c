#include "mpu.h"

// WIRING:
//
// PB8 grey ---- SCL
// PB9 white --- SDA
//

static float m_magn_sensadj[3];

// TEMP_degC   = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21degC 
// Where Temp_degC is the temperature in degrees C measured by the temperature sensor.  
// TEMP_OUT is the actual output of the temperature sensor. 

#define ROOM_TEMP_OFFSET 21 // in C

HAL_StatusTypeDef MPU_get_temp(I2C_HandleTypeDef *hi2c, float *temp) {
	uint8_t rawData[2];  // x/y/z gyro register data stored here

	*temp = 0;
	
	if (MPU_Read(hi2c, REG_TEMP_OUT_H, rawData, 2) != HAL_OK)
		return HAL_ERROR;
			
	*temp = (((uint16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
	
#define TEMP_OFFSET -521
#define TEMP_SENS 340

	// TEMP_degC   = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)  + 21degC
	*temp = 21.0f + ((*temp - (float)TEMP_OFFSET) / TEMP_SENS);

	return HAL_OK;
}

HAL_StatusTypeDef MPU_get_accel(I2C_HandleTypeDef *hi2c, float *accel_data) {
	uint8_t rawData[6];

	if (MPU_Read(hi2c, REG_ACCEL_XOUT_H, rawData, 6) != HAL_OK)
		return HAL_ERROR;
	
	accel_data[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	accel_data[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
	accel_data[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

	accel_data[0] = accel_data[0] * MPU9250_ACCEL_2G_RESOLUTION;
	accel_data[1] = accel_data[1] * MPU9250_ACCEL_2G_RESOLUTION;
	accel_data[2] = accel_data[2] * MPU9250_ACCEL_2G_RESOLUTION;
	
	return HAL_OK;
}

HAL_StatusTypeDef MPU_get_gyro(I2C_HandleTypeDef *hi2c, float *gyro_data) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	
	if (MPU_Read(hi2c, REG_GYRO_XOUT_H, rawData, 6) != HAL_OK)
		return HAL_ERROR;
	
	gyro_data[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	gyro_data[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
	gyro_data[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

	gyro_data[0] = (gyro_data[0] + 630)* MPU9250_GYRO_250DPS_RESOLUTION;
	gyro_data[1] = (gyro_data[1] - 550) * MPU9250_GYRO_250DPS_RESOLUTION;
	gyro_data[2] = (gyro_data[2] + 130)* MPU9250_GYRO_250DPS_RESOLUTION;
		
	return HAL_OK;
}


HAL_StatusTypeDef MPU_get_magn(I2C_HandleTypeDef *hi2c, float *magn_data) {
	uint8_t rawData[AK8963_FRAME_SZ];  // x/y/z gyro register data stored here
		
	// Reading sample data from the slave device, it should be magn data
	if (MPU_Read(hi2c, MPU9250_SLAVE0_DATA_ADDR, rawData, AK8963_FRAME_SZ) != HAL_OK)
		return HAL_ERROR;
	
	// HIGHT byte, than LOW byte
	magn_data[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[2]);
	magn_data[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[4]); 
	magn_data[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[6]);

	magn_data[0] *= m_magn_sensadj[0] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_X;
	magn_data[1] *= m_magn_sensadj[1] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_Y;
	magn_data[2] *= m_magn_sensadj[2] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_Z;

	return HAL_OK;
}
//

HAL_StatusTypeDef MPU_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data, uint8_t bytes_to_read) {
	HAL_StatusTypeDef res;

	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, reg_addr, 1, data, bytes_to_read, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C 0x%x@0x%x: read: failed\r\n", reg_addr, MPU9250_ADDRESS_R);
		return HAL_ERROR;
	}

#ifdef DEBUG_I2C
	if (bytes_to_read == 1)
		printf("DEBUG: I2C 0x%x@0x%x: read: 0x%x\r\n", reg_addr, MPU9250_ADDRESS_R, (uint8_t)*data);
	else
		printf("DEBUG: I2C 0x%x@0x%x: read: %d bytes\r\n", reg_addr, MPU9250_ADDRESS_R, bytes_to_read);
#endif
	
	return HAL_OK;
}

HAL_StatusTypeDef MPU_Write(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data) {
	uint8_t t = data;
	HAL_StatusTypeDef res;

	res = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, reg_addr, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C 0x%x@0x%x: write: 0x%x: failed\r\n", reg_addr, MPU9250_ADDRESS_W, t);
		return HAL_ERROR;
	}

#ifdef DEBUG_I2C
	printf("DEBUG: I2C 0x%x@0x%x: write: 0x%x\r\n", reg_addr, MPU9250_ADDRESS_W, t);
#endif
	
	return HAL_OK;
}


HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c, uint8_t hard_reset_required) {
	uint8_t t;

	// PWR_MGMT_1: Perform hard reset
	if (hard_reset_required) {
		
		if (MPU_Write( hi2c, MPU9250_REG107_ADDR, MPU9250_REG107_H_RESET|MPU9250_REG107_CLKSEL_INTERNAL ) != HAL_OK) {
			// TODO perform low level initialization
			printf("ERROR: I2C: hard reset failed\r\n");
			return HAL_ERROR;
		}

		HAL_Delay(100);
	}
	
	// =========================================================================================
	// Wake up device 
	// PWR_MGMT_1 - reg 107
	// =========================================================================================

	// Clear sleep mode bit (6), enable all sensors 
	// Reset sensors
	if (MPU_Write( hi2c, MPU9250_REG107_ADDR, MPU9250_REG107_CLKSEL_INTERNAL ) != HAL_OK)
			return HAL_ERROR;
	HAL_Delay(100);

	// Auto select clock source to be PLL gyroscope reference if ready else 
	if (MPU_Write( hi2c, MPU9250_REG107_ADDR, MPU9250_REG107_CLKSEL_PLL ) != HAL_OK)
			return HAL_ERROR;
	HAL_Delay(100);

	// =========================================================================================
	// get WHOAMI ID
	// =========================================================================================
	if (MPU_Read(hi2c, MPU9250_REG117_ADDR, &t, 8) != HAL_OK) {
		printf("ERROR: Can't get MPU ID\r\n");
		return HAL_ERROR;
	}
	if ( (t&0xff) != MPU9250_REG117_WAI_ID) {
		printf("ERROR: device has invalid device ID=0x%x, default=0x%x (I2C)\r\n", (t&0xff), MPU9250_REG117_WAI_ID);
		return HAL_ERROR;
	}
			
	// =========================================================================================
	// Configure Gyro and Thermometer
	// =========================================================================================
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	if (MPU_Write(hi2c, MPU9250_REG26_CONFIG_ADDR, MPU9250_REG26_FIFO_NOT_OVERWRITE|MPU9250_REG26_DLPF_CFG) != HAL_OK)
			return HAL_ERROR;
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate determined inset in CONFIG above 
	if (MPU_Write(hi2c, MPU9250_REG25_SMPLRT_DIV_ADDR, MPU9250_REG25_SMPLRT_DIV_DEFAULT) != HAL_OK)
			return HAL_ERROR;

 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	if (MPU_Read(hi2c, MPU9250_REG27_GYRO_ADDR, &t, 1) != HAL_OK)
			return HAL_ERROR;

  t = t & ~MPU9250_REG27_GYRO_SELFTEST_MASK; // Clear self-test bits [7:5]
	t = t & ~MPU9250_REG27_GYRO_FS_MASK; // Clear AFS bits [4:3]
	t = t & ~MPU9250_REG27_GYRO_FCHOICE_MASK; // Clear Fchoice bits [1:0]
	t = t |  MPU9250_REG27_GYRO_FS_250DPS; // Set full scale range for the gyro - 0=+250dps
	
	if (MPU_Write(hi2c, MPU9250_REG27_GYRO_ADDR, t) != HAL_OK)
			return HAL_ERROR;

	
	// =========================================================================================
	// Configure Accelerometer
	// =========================================================================================
	// Set accelerometer full-scale range configuration
	if (MPU_Read(hi2c, MPU9250_REG28_ACCEL_ADDR, &t, 1) != HAL_OK)
			return HAL_ERROR;

  t = t & ~MPU9250_REG28_ACCEL_SELFTEST_MASK; // Clear self-test bits [7:5]
  t = t & ~MPU9250_REG28_ACCEL_CFG_FS_MASK;  // Clear AFS bits [4:3]
  t = t |  MPU9250_REG28_ACCEL_FS_2G; // Set full scale range for the accelerometer  - 0=2g
	
	if (MPU_Write(hi2c, MPU9250_REG28_ACCEL_ADDR, t) != HAL_OK)
			return HAL_ERROR;

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz 	
	if (MPU_Read(hi2c, MPU9250_REG29_ACCEL2_ADDR, &t, 1) != HAL_OK)
			return HAL_ERROR;

	t = t & ~MPU9250_REG29_ACCEL2_FCHOICE_MASK; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) 
  t = t |  MPU9250_REG29_ACCEL2_DLPFCFG;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	
	if (MPU_Write(hi2c, MPU9250_REG29_ACCEL2_ADDR, t) != HAL_OK)
			return HAL_ERROR;
	
	// =========================================================================================
	// Setip MP9250 to use compass via internal i2c bus
	// =========================================================================================

	// Disable by-pass mode (INT_PIN_CFG @55)
	// Disconnects the bypass on EDA/ECL `	if it was enabled
	if (MPU_Read(hi2c, MPU9250_REG55_ADDR, &t, 1) != HAL_OK)
		return HAL_ERROR;	
	t = t & ~MPU9250_REG55_BYPASS_EN;
	if (MPU_Write(hi2c, MPU9250_REG55_ADDR, t) != HAL_OK)
		return HAL_ERROR;	

	// Enables the I2C Master within the MPU (USER_CTRL @106)
	if (MPU_Read(hi2c, MPU9250_REG106_ADDR, &t, 1) != HAL_OK)
		return HAL_ERROR;	
	t = t | MPU9250_REG106_FIFO_EN;
	t = t | MPU9250_REG106_I2C_MST_EN;
	if (MPU_Write(hi2c, MPU9250_REG106_ADDR, t) != HAL_OK)
		return HAL_ERROR;	
	
	
	// Setup I2C MST 
	if (MPU_Read(hi2c, MPU9250_REG36_ADDR, &t, 1) != HAL_OK)
		return HAL_ERROR;	
	
	// clear MULT_MST_EN [7] - Unless there are multiple masters hooked up to that bus
	t = t & ~MPU9250_REG36_MULT_MST_EN;
	// clear SLV3_FIFO_EN [5] - Unless you are using FIFO for Slave3 requested data 
	t = t & ~MPU9250_REG36_SLV_3_FIFO_EN;
	// enable WAIT_FOR_ES [6] - Causes the data ready interrupt to wait for the external data
	t = t | MPU9250_REG36_WAIT_FOR_ES;
	// enable I2C_P_NSR [4] - When transitioning slaves; issue a full stop, then a start
	t = t | MPU9250_REG36_I2C_MST_P_NSR;
	// I2C_MST_CLK = 400kHz 
	t = t | MPU9250_REG36_I2C_MST_CLK_400KHZ;
	
	if (MPU_Write(hi2c, MPU9250_REG36_ADDR, t) != HAL_OK)
		return HAL_ERROR;	
	
	// ---------------------------------
	// DISABLE SLAVES #1,2,3
	// ---------------------------------
	
	// DISABLE SLAVE1
	// Register 42 - I2C_SLV1_CTRL 
	if (MPU_Write(hi2c, MPU9250_REG42_ADDR, MPU9250_REG42_DISABLE) != HAL_OK)
		return HAL_ERROR;	
	
	// Register 45 - I2C_SLV2_CTRL 
	// DISABLE SLAVE2
	if (MPU_Write(hi2c, MPU9250_REG45_ADDR, MPU9250_REG45_DISABLE) != HAL_OK)
		return HAL_ERROR;	

	// Register 48 - I2C_SLV3_CTRL 
	// DISABLE SLAVE2
	if (MPU_Write(hi2c, MPU9250_REG48_ADDR, MPU9250_REG48_DISABLE) != HAL_OK)
		return HAL_ERROR;	

	
	// ---------------------------------
	// MAGN
	// ---------------------------------

	// AK8963 - Get Device ID
	if (AK8963_SL0_Read(hi2c, AK8963_WAI_ADDR, &t, 1) != HAL_OK)
		return HAL_ERROR;

	if ( (t&0xff) != AK8963_REG0_WAI_ID) {
		printf("ERROR: device has invalid AK8963 device ID=0x%x, default=0x%x (I2C)\r\n", (t&0xff), AK8963_REG0_WAI_ID);
		return HAL_ERROR;
	}
	
	// Shutdown AK8963
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0x0) != HAL_OK)
		return HAL_ERROR;
	HAL_Delay(100);
	
	// set AK8963 to FUSE ROM access
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0xF) != HAL_OK)
		return HAL_ERROR;
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	uint8_t buffer[3];
	if (AK8963_SL0_Read(hi2c, 0x10, buffer, 3) != HAL_OK)
		return HAL_ERROR;
	
	m_magn_sensadj[0] = (((float)buffer[0] - 128.0f)/(256.0f) + 1.0f);
	m_magn_sensadj[1] = (((float)buffer[1] - 128.0f)/(256.0f) + 1.0f);
	m_magn_sensadj[2] = (((float)buffer[2] - 128.0f)/(256.0f) + 1.0f);
	
	printf("DEBUG: MGN Raw Adj: x=%d y=%d z=%d\r\n", 
		buffer[0], buffer[1], buffer[2]);

	printf("DEBUG: MGN SensAdj: x=%f y=%f z=%f\r\n", 
		m_magn_sensadj[0], m_magn_sensadj[1], m_magn_sensadj[2]);
	
	// Shutdown AK8963
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0x0) != HAL_OK)
		return HAL_ERROR;
	HAL_Delay(100);
	
	// ---------------------------------
	// Magn :: Power On
	// ---------------------------------
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0x16) != HAL_OK)
		return HAL_ERROR;
	HAL_Delay(100);

	// ---------------------------------
	// Magn :: Setup MAGN as a source for Slave0
	// ---------------------------------
	if (AK8963_SL0_Setup(hi2c, AK8963_HXL_ADDR, AK8963_FRAME_SZ) != HAL_OK)
		return HAL_ERROR;

	// =========================================================================================
	// Setup MPU9250 - FIFO
	// =========================================================================================
//	if (MPU_Write(hi2c, MPU9250_REG35_ADDR, MPU9250_REG35_DEFAULT) != HAL_OK)
//		return HAL_ERROR;	
//	printf("DEBUG: FIFO(%d): 0x%x\r\n", MPU9250_REG35_ADDR, t);

	// =========================================================================================
	// BYPASS_EN 
	// =========================================================================================
	//	if (MPU_Write(hi2c, MPU9250_REG55_ADDR, MPU9250_REG55_ENABLE_BYPASS) != HAL_OK)
	//	return HAL_ERROR;	
	
	printf("DEBUG: MPU initliazed sucessully\r\n");
	return HAL_OK;
}
//

// using SLAVE0 to write data
HAL_StatusTypeDef AK8963_Write(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t data) {
	
	return HAL_OK;
}
//


HAL_StatusTypeDef AK8963_Read(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data, uint8_t bytes_to_read) {
	HAL_StatusTypeDef res;

	
	for(int i = 1; i < 127; i++) {
		res = HAL_I2C_Mem_Read(hi2c, i, 0x0, 1, data, bytes_to_read, MPU_I2C_TIMEOUT);
		if (res == HAL_OK)
				printf("FOUND @0x%x\r\n", i);
	}
	
//	res = HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS_R, reg_addr, 1, data, bytes_to_read, MPU_I2C_TIMEOUT);
	res = HAL_I2C_Mem_Read(hi2c, 0x0c<<1|1, reg_addr, 1, data, bytes_to_read, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C 0x%x@0x%x: read: failed\r\n", reg_addr, AK8963_ADDRESS_R);
		return HAL_ERROR;
	}

#ifdef DEBUG_I2C
	if (bytes_to_read == 1)
		printf("DEBUG: I2C 0x%x@0x%x: read: 0x%x\r\n", reg_addr, AK8963_ADDRESS_R, (uint8_t)*data);
	else
		printf("DEBUG: I2C 0x%x@0x%x: read: %d bytes\r\n", reg_addr, AK8963_ADDRESS_R, bytes_to_read);
#endif
	
	return HAL_OK;
}



// using SLAVE0 to write data
HAL_StatusTypeDef AK8963_SL0_Write(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t data) {
	uint8_t t;
	
	// setup write mode
	t = MPU9250_REG37_AK8963_WRITE;
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_SLAVE0_BASE, 1, &t, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;

	// setup address 
	t = ak8963_reg_addr;
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_SLAVE0_BASE+1, 1, &t, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	
	// Register 99 I2C Slave 0 Data Out 
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, 99, 1, &data, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;

	printf("DEBUG: AK8963: write: 0x%x to AK8369:0x%x (local 0x%x)\r\n", data, ak8963_reg_addr, 99);
	
	return HAL_OK;
}
//

// Using SLAVE0 to read one byte
HAL_StatusTypeDef AK8963_SL0_Setup(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t bytes_to_read) {
	uint8_t t;
	
	// Setup I2C slave #0
	t = MPU9250_REG37_I2C_SLV0_RNW_READ | AK8963_ADDRESS_R;
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_SLAVE0_BASE, 1, &t, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	
	t = ak8963_reg_addr;
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_SLAVE0_BASE+1, 1, &t, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	
	// Enable reading data from this slave at the sample rate 
	t = bytes_to_read | MPU9250_REG39_I2C_SLV0_EN;
	if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_SLAVE0_BASE+2, 1, &t, 1, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	
	return HAL_OK;
}

HAL_StatusTypeDef AK8963_SL0_Read(I2C_HandleTypeDef *hi2c, uint8_t ak8963_reg_addr, uint8_t *data, uint8_t bytes_to_read) {
	
	// reading 1 byte
	if (AK8963_SL0_Setup(hi2c, ak8963_reg_addr, bytes_to_read) != HAL_OK)
		return HAL_ERROR;
		
	HAL_Delay(50);
	
	// Reading 0x0 from slave device (73 register)
	if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_SLAVE0_DATA_ADDR, 1, data, bytes_to_read, MPU_I2C_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	
	printf("DEBUG: AK8963: read (1 byte): 0x%x from AK8369:0x%x (local 0x%x)\r\n", *data, ak8963_reg_addr, MPU9250_SLAVE0_DATA_ADDR);
	
	return HAL_OK;
}

int16_t MPU_GetFifoCnt( I2C_HandleTypeDef *hi2c ) {
	HAL_StatusTypeDef res;
	uint8_t t;
	int16_t t_fifo_cnt = 0;

	// Read Register 114 and 115 - FIFO Count Registers 
	
	// FIFO_CNT[12:8] 4 bits
	// High Bits, count indicates the number of written bytes in the FIFO. 
	// Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL. 
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, 114, 1, &t, 1, MPU_I2C_TIMEOUT); 	
	if (res != HAL_OK) {
		return -1;
	}
	t_fifo_cnt = (uint16_t) (t & 0x0F);

	// FIFO_CNT[7:0] 
	// Low Bits, count indicates the number of written bytes in the  FIFO.
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, 115, 1, &t, 1, MPU_I2C_TIMEOUT); 	
	if (res != HAL_OK) {
		return -1;
	}
	t_fifo_cnt = (t_fifo_cnt << 8) + (uint16_t)(t & 0xFF);
	
	// it has 512 bytes buffer only
	
	// printf("DEBUG: FIFO: bytes available: %d\r\n", t_fifo_cnt);
	
	return t_fifo_cnt;
}
//	

HAL_StatusTypeDef MPU_ResetFifo( I2C_HandleTypeDef *hi2c ) {
	uint8_t t;
	
	// Register 106 - User Control 
	// [2]  FIFO_RST 
	// 1 - Reset FIFO module. Reset is asynchronous.  This bit auto clears after  one clock cycle. 
	if (MPU_Read(hi2c, MPU9250_REG106_ADDR, &t, 1) != HAL_OK)
			return HAL_ERROR;
	
  t |= MPU9250_REG106_FIFO_RST;
	
  // Write new ACCEL_CONFIG register value
	if (MPU_Write(hi2c, MPU9250_REG106_ADDR, t) != HAL_OK)
			return HAL_ERROR;
	
	HAL_Delay(100);
  	
	return HAL_OK;
}
//


HAL_StatusTypeDef MPU_GetFifoFrameData( I2C_HandleTypeDef *hi2c, float *accel_data, float *gyro_data, float *magn_data, float *temp ) {
	HAL_StatusTypeDef res;
	uint8_t t;
	uint16_t data[MPU9250_FIFO_FRAME_SIZE];
	
		int16_t t_fifo_cnt = MPU_GetFifoCnt( hi2c );
		if (t_fifo_cnt < 0) {
			printf("ERROR: MPU_GetData: can't read data\r\n");
			return HAL_ERROR;
		}	
		
		if (t_fifo_cnt < MPU9250_FIFO_FRAME_SIZE) {
#ifdef DEBUG_I2C			
			printf("DEBUG: need more data to read: %d of %d bytes available\r\n", t_fifo_cnt, MPU9250_FIFO_FRAME_SIZE);
#endif
			return HAL_OK;
		}
		
#ifdef DEBUG_I2C		
		printf("DEBUG: %d (framesize %d) bytes to read\r\n", t_fifo_cnt, MPU9250_FIFO_FRAME_SIZE);
#endif
	
	// Register 116 - FIFO Read Write
	// Read/Write command provides Read or Write operation for  the FIFO.  
	// Description: 
	// This register is used to read and write data from the FIFO buffer.  
	// Data is written to the FIFO in order of register number (from lowest to highest). If all the FIFO enable 
	// flags  (see  below)  are  enabled  and  all  External  Sensor  Data  registers  (Registers  73  to  96)  are 
	// associated with a Slave device, the contents of registers 59 through 96 will be written in order at the 
	// Sample Rate. 
	// The contents of the sensor data registers (Registers 59 to 96) are written into the FIFO buffer when 
	// their corresponding FIFO enable flags are set to 1 in FIFO_EN (Register 35). An additional flag for 
	// the sensor data registers associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).  
	// If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is automatically set to 1. This bit 
	// is located in INT_STATUS (Register 58). When the FIFO buffer has overflowed, the oldest data will 
	// be lost and new data will be written to the FIFO unless register 26 CONFIG, bit[6] FIFO_MODE = 1.
	// If the FIFO buffer is empty, reading this register will return the last byte that was previously read from 
	// the FIFO until new data is available. The user should check FIFO_COUNT to ensure that the FIFO 
	// buffer is not read when empty. 

	for (int i = 0; i < MPU9250_FIFO_FRAME_SIZE; i++) {
		
	if (MPU_Read(hi2c, MPU9250_REG116_ADDR, &t, 1) != HAL_OK)
			return HAL_ERROR;
			
		res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, 116, 1, &t, 1, MPU_I2C_TIMEOUT); 	
		if (res != HAL_OK) {
			return HAL_ERROR;
		}
		
		data[i] = t;
	}
	
	// ACCE: x[0:1] y[2:3] z[4:5]
	accel_data[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
	accel_data[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
	accel_data[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);

	accel_data[0] = accel_data[0] * MPU9250_ACCEL_2G_RESOLUTION;
	accel_data[1] = accel_data[1] * MPU9250_ACCEL_2G_RESOLUTION;
	accel_data[2] = accel_data[2] * MPU9250_ACCEL_2G_RESOLUTION;
	
	// TEMP: [6:7]
	*temp = (float) (((uint16_t)data[6]) << 8 | data[7]);  // Turn the MSB and LSB into a 16-bit value
	// TEMP_degC   = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)  + 21degC
	#define TEMP_OFFSET -521
	#define TEMP_SENS 340
	*temp = 21.0f + ((*temp - (float)TEMP_OFFSET) / TEMP_SENS);

	// GYRO: x[8:9] y[10:11] z[12:13]
	gyro_data[0] = (int16_t)(((int16_t)data[8] << 8) | data[9]);  
	gyro_data[1] = (int16_t)(((int16_t)data[10] << 8) | data[11]); 
	gyro_data[2] = (int16_t)(((int16_t)data[12] << 8) | data[13]);
	
	gyro_data[0] = gyro_data[0] * MPU9250_GYRO_250DPS_RESOLUTION;
	gyro_data[1] = gyro_data[1] * MPU9250_GYRO_250DPS_RESOLUTION;
	gyro_data[2] = gyro_data[2] * MPU9250_GYRO_250DPS_RESOLUTION;
	
	// SLAVE1: MAGN DATA: ST1[14] x[16:15] y[18:17] z[20:19] ST2[21]

	// checking AT8936 ST1 registger: 

	// DRDY bit turns to 1 when data is ready in single measurement mode or self-test mode. 
	// It returns to 0 when any one of ST2 register or measurement data register (HXL to HZH) is read.
	if (data[14] & 0x01) {
#ifdef DEBUG_I2C		
		// IGNORE IT NOW
		printf("WARNING: AK8963: ST1: 0x%x: DRDY (01b): Data is not ready: device is in signle measurement or self-test mode)\r\n", data[14]);
#endif
	}
	
	// 
	if (data[14] & 0x2) {
#ifdef DEBUG_I2C		
		printf("WARNING: AK8963: ST1: 0x%x: DOR (10b): Data overrun\r\n", data[14]);
#endif
	}
	
	// 14:HXL 15:HXH
	magn_data[0] = (float)(((int16_t)data[16] << 8) | data[15]);
	magn_data[1] = (float)(((int16_t)data[18] << 8) | data[17]); 
	magn_data[2] = (float)(((int16_t)data[20] << 8) | data[19]);
	
	magn_data[0] *= m_magn_sensadj[0] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_X;
	magn_data[1] *= m_magn_sensadj[1] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_Y;
	magn_data[2] *= m_magn_sensadj[2] * AK8963_SCALE_FACTOR + AK8963_MAG_BIAS_Z;

	// checking AT8936 ST2 registger: 
	// HOFL: Magnetic sensor overflow [3]
	if (data[21] & 0x8) {
#ifdef DEBUG_I2C				
		// TODO
		printf("WARNING: AK8963: ST2: 0x%x: Magnetic sensor overflow\r\n", data[21]);
#endif
	}
//	printf("DEBUG: Temp %f\r\n", *temp);

	return HAL_OK;
}
//

HAL_StatusTypeDef MPU_GetData( I2C_HandleTypeDef *hi2c, float *accel_data, float *gyro_data, float *magn_data, float *temp ) {
	
	MPU_get_temp( hi2c, temp );
	MPU_get_accel( hi2c, accel_data);
	MPU_get_gyro( hi2c, gyro_data);
	MPU_get_magn( hi2c, magn_data);
	
	return HAL_OK;
}
