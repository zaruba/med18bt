#include "mpu.h"

// WIRING:
//
// PB8 grey ---- SCL
// PB9 white --- SDA
//

static float m_magn_sensadj[3];

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

	
void MPU_get_magn(I2C_HandleTypeDef *hi2c, int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
		
	// Reading sample data from the slave device, it should be magn data
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_SLAVE0_DATA_ADDR, 1, (uint8_t *)rawData, 6, MPU_I2C_TIMEOUT);
	// HIGHT byte, than LOW byte
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]); 
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}
//

	
HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef res;
	uint8_t t;
	
	// =========================================================================================
	// Setup ACCEL/GYRO/TEMP MPU9250
	// =========================================================================================

	// get WHOAMI ID
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_REG117_ADDR, 1, &t, 8, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C 0x%x@0x%x: Can't get MPU ID\r\n", MPU9250_REG117_ADDR, MPU9250_ADDRESS_R);
		return HAL_ERROR;
	}
	if ( (t&0xff) != MPU9250_REG117_WAI_ID) {
		printf("ERROR: device has invalid device ID=0x%x, default=0x%x (I2C)\r\n", (t&0xff), MPU9250_REG117_WAI_ID);
		return HAL_ERROR;
	}
		
	// PWR_MGMT_1:
	// Perform hard reset
	t = MPU9250_REG107_H_RESET; 
	res = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG107_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C failed: hard reset: PWR_MGMT_1 (107)\r\n");
		return HAL_ERROR;
	}
	printf("DEBUG: PWR_MGMT_1 (107): hard reset: 0x%x\r\n", t);
	HAL_Delay(100);

	// Clear sleep mode bit (6), reset and enable all sensors 
	t = MPU9250_REG107_CLKSEL_DEFAULT; 
	res = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG107_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: I2C failed: PWR_MGMT_1 (107)\r\n");
		return HAL_ERROR;
	}
	printf("DEBUG: PWR_MGMT_1 (107): setup: 0x%x\r\n", t);
	HAL_Delay(10);
	
	// ---------------------------------
	// Configure Gyro and Accelerometer
	// ---------------------------------

	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	t = MPU9250_REG26_DEFAULT;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG26_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("ERROR: Can't init MPU: DLPF_CFG (I2C)\r\n");
		return HAL_ERROR;
	}
	printf("DEBUG: DLPF_CFG (26): 0x%x\r\n", t);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	t = MPU9250_REG25_SMPLRT_DIV_DEFAULT;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG25_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);   // Use a 200 Hz rate; the same rate set in CONFIG above
	printf("DEBUG: SMPLRT_DIV (25): 0x%x\r\n", t);
   
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

	// =========================================================================================
	// Setip MP9250 to use compass via internal i2c bus
	// =========================================================================================

	// Disable by-pass mode (INT_PIN_CFG @55)
	// Disconnects the bypass on EDA/ECL `	if it was enabled
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_REG55_ADDR, 1, &t, 6, MPU_I2C_TIMEOUT);
	t = t & ~MPU9250_REG55_BYPASS_EN;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG55_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	printf("DEBUG: INT_PIN_CFG (55): 0x%x\r\n", t);

	// Enables the I2C Master within the MPU (USER_CTRL @106)
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_REG106_ADDR, 1, &t, 6, MPU_I2C_TIMEOUT); 
	t = t | MPU9250_REG106_FIFO_EN;
	t = t | MPU9250_REG106_I2C_MST_EN;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG106_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	printf("DEBUG: USER_CTRL (106): 0x%x\r\n", t);

	// Setup I2C MST 
	res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_REG36_ADDR, 1, &t, 6, MPU_I2C_TIMEOUT); 
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
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG36_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	printf("DEBUG: I2C_MST_CTRL(%d): 0x%x\r\n", MPU9250_REG36_ADDR, t);

	// ---------------------------------
	// DISABLE SLAVES #1,2,3
	// ---------------------------------
	
	// DISABLE SLAVE1
	// Register 42 - I2C_SLV1_CTRL 
	t = MPU9250_REG42_DISABLE;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG42_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	
	// Register 45 - I2C_SLV2_CTRL 
	// DISABLE SLAVE2
	t = MPU9250_REG45_DISABLE;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG45_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);

	// Register 48 - I2C_SLV3_CTRL 
	// DISABLE SLAVE2
	t = MPU9250_REG48_DISABLE;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG48_ADDR , 1, &t, 1, MPU_I2C_TIMEOUT);

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
	HAL_Delay(50);
	
	// set AK8963 to FUSE ROM access
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0xF) != HAL_OK)
		return HAL_ERROR;
	HAL_Delay(50);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	uint8_t buffer[3];
	if (AK8963_SL0_Read(hi2c, 0x10, buffer, 3) != HAL_OK)
		return HAL_ERROR;

	m_magn_sensadj[0] = (((float)buffer[0] - 128.0f)/(256.0f) + 1.0f) * AK8963_SCALE_FACTOR;
	m_magn_sensadj[1] = (((float)buffer[1] - 128.0f)/(256.0f) + 1.0f) * AK8963_SCALE_FACTOR;
	m_magn_sensadj[2] = (((float)buffer[2] - 128.0f)/(256.0f) + 1.0f) * AK8963_SCALE_FACTOR;
	
	printf("DEBUG: MGN SensAdj: x=%f y=%f z=%f\r\n", 
		m_magn_sensadj[0], m_magn_sensadj[1], m_magn_sensadj[2]);
	
	// ---------------------------------
	// Magn :: Power On
	// ---------------------------------
	if (AK8963_SL0_Write(hi2c, 0xA, (uint8_t)0x16) != HAL_OK)
		return HAL_ERROR;

	// ---------------------------------
	// Magn :: Setup MAGN as a source for Slave0
	// ---------------------------------
	if (AK8963_SL0_Setup(hi2c, AK8963_HXL_ADDR, 7) != HAL_OK)
		return HAL_ERROR;

	// =========================================================================================
	// Setup MPU9250 - FIFO
	// =========================================================================================
	t = MPU9250_REG35_DEFAULT;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG35_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);
	printf("DEBUG: FIFO(%d): 0x%x\r\n", MPU9250_REG35_ADDR, t);
	
	
	printf("DEBUG: MPU initliazed sucessully\r\n");
	return HAL_OK;
}
//

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
	
	// Register 99 – I2C Slave 0 Data Out 
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
		
	HAL_Delay(10);
	
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

	// Read Register 114 and 115 – FIFO Count Registers 
	
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
	HAL_StatusTypeDef res;
	uint8_t t;
	
	// Register 106 – User Control 
	// [2]  FIFO_RST 
	// 1 – Reset FIFO module. Reset is asynchronous.  This bit auto clears after  one clock cycle. 
  res = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS_R, MPU9250_REG106_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);   // get current ACCEL_CONFIG register value
	if (res != HAL_OK)
		return HAL_ERROR;

  t |= MPU9250_REG106_FIFO_RST;
  res = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS_W, MPU9250_REG106_ADDR, 1, &t, 1, MPU_I2C_TIMEOUT);   // Write new ACCEL_CONFIG register value
	if (res != HAL_OK)
		return HAL_ERROR;
  	
	return HAL_OK;
}
//


HAL_StatusTypeDef MPU_GetFifoFrameData( I2C_HandleTypeDef *hi2c, int16_t *accel_data, int16_t *gyro_data, float *magn_data, float *temp ) {
	HAL_StatusTypeDef res;
	uint8_t t;
	uint16_t data[MPU9250_FIFO_FRAME_SIZE];
	
		int16_t t_fifo_cnt = MPU_GetFifoCnt( hi2c );
		if (t_fifo_cnt < 0) {
			printf("ERROR: MPU_GetData: can't read data\r\n");
			return HAL_ERROR;
		}	
		
		if (t_fifo_cnt < MPU9250_FIFO_FRAME_SIZE) {
			printf("DEBUG: need more data to read: %d of %d bytes available\r\n", t_fifo_cnt, MPU9250_FIFO_FRAME_SIZE);
			return HAL_OK;
		}
					
		// printf("DEBUG: %d (framesize %d) bytes to read\r\n", t_fifo_cnt, MPU9250_FIFO_FRAME_SIZE);

	
	// Register 116 – FIFO Read Write
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

	// TEMP: [6:7]
	*temp = (float) (((uint16_t)data[6]) << 8 | data[7]);  // Turn the MSB and LSB into a 16-bit value
	// TEMP_degC   = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity)  + 21degC
	#define TEMP_OFFSET -521
	#define TEMP_SENS 340
	*temp = 21 + ((*temp - (float)TEMP_OFFSET) / TEMP_SENS);
	// *temp = *temp / 100.0f;

	// GYRO: x[8:9] y[10:11] z[12:13]
	gyro_data[0] = (int16_t)(((int16_t)data[8] << 8) | data[9]);  // Turn the MSB and LSB into a signed 16-bit value
	gyro_data[1] = (int16_t)(((int16_t)data[10] << 8) | data[11]); 
	gyro_data[2] = (int16_t)(((int16_t)data[12] << 8) | data[13]);
	
	// SLAVE1: MAGN DATA: x[15:14] y[17:16] z[19:18]
	// 14:HXL 15:HXH
	magn_data[0] = (float)(((int16_t)data[15] << 8) | data[14]);
	magn_data[1] = (float)(((int16_t)data[17] << 8) | data[16]); 
	magn_data[2] = (float)(((int16_t)data[19] << 8) | data[18]);
	
	magn_data[0] *= m_magn_sensadj[0];
	magn_data[1] *= m_magn_sensadj[1];
	magn_data[2] *= m_magn_sensadj[2];

	// checking AT8936 ST2 registger: 
	// HOFL: Magnetic sensor overflow [3]
	if (data[20] & 0x8) {
		printf("WARNING: ST2: 0x%x: Magnetic sensor overflow\r\n", data[20]);
	}
//	printf("DEBUG: Temp %f\r\n", *temp);

	return HAL_OK;
}
//

HAL_StatusTypeDef MPU_Init_MAGN(I2C_HandleTypeDef *hi2c) {
  
	//uint8_t t;
//	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	//HAL_StatusTypeDef res;

/*	
	// First extract the factory calibration for each magnetometer axis
	R=0x00;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, MPU_I2C_TIMEOUT);    // Power down magnetometer 
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
*/
	return HAL_OK;
}
//
