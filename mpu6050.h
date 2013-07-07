/* Copyright (C) 2013 Enrico Rossi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*! \file mpu6050.h */

#ifndef MPU6050
#define MPU6050

#include "i2c.h"

/** The address of the device.
 * Typical it is 0x68 or 0x69 in a 7bit notation.
 * If the AD0 pin is low then it is 0x68 else it is
 * 0x69.
 * You have to shift left the address to add the LSB
 * which is used for READ/WRITE operation, thus
 * the address is 0xD0 or 0xD2.
 */
#define MPU6050_ADDR 0xD0 /* AD0 low */
#define MPU6050_RA_WHO_AM_I 0x75
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20

#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A

#define MPU6050_RA_ACCEL_XOUT 0x3B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT 0x3D
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT 0x3F
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT 0x41
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT 0x43
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT 0x45
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT 0x47
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C

#define MPU6050_PWR1_DEVICE_RESET_BIT 7
#define MPU6050_PWR1_SLEEP_BIT 6
#define MPU6050_PWR1_CYCLE_BIT 5
#define MPU6050_PWR1_TEMP_DIS_BIT 3
#define MPU6050_PWR1_CLKSEL_BIT 2
#define MPU6050_PWR1_CLKSEL_LENGTH 3

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT 7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH 2
#define MPU6050_PWR2_STBY_XA_BIT 5
#define MPU6050_PWR2_STBY_YA_BIT 4
#define MPU6050_PWR2_STBY_ZA_BIT 3
#define MPU6050_PWR2_STBY_XG_BIT 2
#define MPU6050_PWR2_STBY_YG_BIT 1
#define MPU6050_PWR2_STBY_ZG_BIT 0

#define MPU6050_FLAG_CALIBRATED 0
#define MPU6050_FLAG_COM_ERR 0xf
/** Number of cycle to calibrate the gyro. */
#define MPU6050_GYRO_CALIBRATION 0x40

struct mpu6050_t {
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int offset_gx;
	int offset_gy;
	int offset_gz;
	float temp;
	uint16_t flags;
};

uint8_t mpu6050_init(struct mpu6050_t *mpu6050);
uint8_t mpu6050_read_all(struct mpu6050_t *mpu6050);
uint8_t mpu6050_LPA(uint8_t mode, struct mpu6050_t *mpu6050);
uint8_t mpu6050_read_irq(uint8_t *byte);

#endif
