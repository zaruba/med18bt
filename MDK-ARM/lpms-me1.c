#include "lpms-me1.h"

HAL_StatusTypeDef LPMS_ME1_Init(I2C_HandleTypeDef *hi2c) {
	return HAL_OK;
}

/**
  *@brief: Get euler angle
  *@para: Pointer to float array that have 3 elements at least
  *@ret: Status, return HAL_OK if read success otherwise return HAL_ERROR
  */

// PB8 = сер = SCL = #9
// PB9 = бел = SDA = #7
HAL_StatusTypeDef LPMS_ME1_GetEuler(I2C_HandleTypeDef *hi2c, float *euler) {
	LPMS_ME1_DataDecoder data[3];
	uint8_t t = 0x0;
	HAL_StatusTypeDef res;

//	res = HAL_I2C_Mem_Read(hi2c, LPMS_ME1_I2C_ADRRESS, LPMS_ME1_WAI, 1, &t, 8, LPMS_ME1_I2C_TIMEOUT);
//	res = HAL_I2C_Mem_Read(hi2c, 0xD2, 117, 1, &t, 8, LPMS_ME1_I2C_TIMEOUT);

	res = HAL_I2C_Mem_Read(hi2c, 0x64, 0x74, I2C_MEMADD_SIZE_8BIT, &t, 1, LPMS_ME1_I2C_TIMEOUT);
	if (res != HAL_OK) {
		printf("DEBUG: I2C@0x%x:0x%x read failed (%d)\r\n", 0x64, LPMS_ME1_WAI, I2C_MEMADD_SIZE_8BIT);
		//return HAL_ERROR;

		res = HAL_I2C_Mem_Read(hi2c, 0x66, 0x74, I2C_MEMADD_SIZE_8BIT, &t, 1, LPMS_ME1_I2C_TIMEOUT);
		if (res != HAL_OK) {
			printf("DEBUG: I2C@0x%x:0x%x read failed\r\n", 0x66, LPMS_ME1_WAI);

			res = HAL_I2C_Mem_Read(hi2c, 0x32, 0x74, I2C_MEMADD_SIZE_8BIT, &t, 1, LPMS_ME1_I2C_TIMEOUT);
			if (res != HAL_OK) {
				printf("DEBUG: I2C@0x%x:0x%x read failed\r\n", 0x32, LPMS_ME1_WAI);
				
				res = HAL_I2C_Mem_Read(hi2c, 0x33, 0x74, I2C_MEMADD_SIZE_8BIT, &t, 1, LPMS_ME1_I2C_TIMEOUT);
				if (res != HAL_OK) {
					printf("DEBUG: I2C@0x%x:0x%x read failed\r\n", 0x33, LPMS_ME1_WAI);
					//return HAL_ERROR;
				}
			}
		}
	}
		
	printf("DEBUG: WAI ID: 0x%x\r\n", t);
	
	if (HAL_I2C_Mem_Read(hi2c, LPMS_ME1_I2C_ADRRESS, LPMS_ME1_EULER_X_0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data[0].u8vals, 12, LPMS_ME1_I2C_TIMEOUT) != HAL_OK) {
		return HAL_ERROR;
	}
	
	for(uint8_t i = 0; i<3; i++) {
		*(euler+i) = data[i].fval;
	}

	return HAL_OK;
}

//
