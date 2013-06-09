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
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C

#define MPU6050_FLAG_CALIBRATED 0
#define MPU6050_FLAG_COM_ERR 0xf
/** Number of cycle to calibrate the gyro. */
#define MPU6050_GYRO_CALIBRATION 0x0f

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

#endif
