#ifndef _MPU_H_
#define _MPU_H_

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

#define REG_CONFIG 0x1a
#define FLG_CONFIG_EXT_SYNC_SET_DISABLED 0x0
#define FLG_CONFIG_DLPF_CFG_41HZ 0x3

#define REG_GYRO_CFG 0x1b
#define FLG_GYRO_CFG_GYRO_FS_250DPS (0x0 << 3) // 
#define MASK_GYRO_CFG_SELFTEST (0x1 << 7 | 0x1 << 6 | 0x1 << 5) // bits 7-5
#define MASK_GYRO_CFG_FCHOICE (0x1 << 1 | 0x1 )
#define MASK_GYRO_CFG_FS (0x1 << 4 | 0x1 < 3)

#define REG_ACCEL_CFG 0x1c
#define MASK_ACCEL_CFG_SELFTEST (0x1 << 7 | 0x1 << 6 | 0x1 << 5) // bits 7-5
#define MASK_ACCEL_CFG_DLPFCFG (0x1 << 1 | 0x1 )
#define MASK_ACCEL_CFG_FS (0x1 << 4 | 0x1 < 3)

#endif // _MPU_H_
