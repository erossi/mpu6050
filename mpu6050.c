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

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "mpu6050.h"

void swapit(uint16_t *swapme)
{
	uint8_t L;

	L = (*swapme) & 0xff;
	(*swapme) = (*swapme) >> 8;
	(*swapme) |= (uint16_t)L << 8;
}

/** Register Read (Byte).
 *
 * @param addr the address of the device.
 * @param reg_addr the register address.
 * @param byte the data to be read.
 */
uint8_t register_rb(const uint8_t addr, const uint8_t reg_addr,
		uint8_t *byte)
{
	uint8_t error;

	error = i2c_master_send_b(addr, reg_addr, FALSE);

	if (!error)
		error = i2c_master_read_b(addr, byte, TRUE);

	return (error);
}

/** Register Read (word).
 *
 * @param addr the address of the device.
 * @param reg_addr the register address.
 * @param byte the data to be read.
 */
uint8_t register_rw(const uint8_t addr, const uint8_t reg_addr,
		uint16_t *word)
{
	uint8_t err;

	err = i2c_master_send_b(addr, reg_addr, FALSE);

	if (!err)
		err = i2c_master_read_w(addr, word);

	/*
		err = i2c_master_read_w(addr, word, TRUE);
		*/

	return (err);
}

/** Register write (Byte).
 *
 * @param addr the address of the device.
 * @param reg_addr the register address.
 * @param byte the data to be written.
 */
uint8_t register_wb(const uint8_t addr, const uint8_t reg_addr,
		uint8_t byte)
{
	uint8_t error;

	error = i2c_master_send_w(addr, reg_addr, byte);
	return (error);
}

/** Init
 * The default id is 0b110100
 */
uint8_t mpu6050_init(void)
{
	uint8_t byte, error;

	i2c_init();

	error = register_rb(MPU6050_ADDR, MPU6050_RA_WHO_AM_I, &byte);

	if (error || (byte != 0x68)) {
		error |= 0x80;
	}

	if (!error) {
		/* Sample rate /8 */
		error |= register_wb(MPU6050_ADDR,
				MPU6050_RA_SMPLRT_DIV, 7);
		error |= register_wb(MPU6050_ADDR, MPU6050_RA_CONFIG, 6);
		/* Full scale */
		error |= register_wb(MPU6050_ADDR,
				MPU6050_RA_GYRO_CONFIG, 0x18);
		error |= register_wb(MPU6050_ADDR,
				MPU6050_RA_ACCEL_CONFIG, 1);
		/* No sleep
		 * clksrc Internal 8KHz
		 * Temp. enable.
		 */
		error = register_wb(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1);
	}

	return (error);
}

/** The temperature read and converter.
 *
 * From the datasheet:
 * at 35 degrees celsius the raw value is -521.
 *
 * T = (Traw/340) + offset
 * where:
 * T is the temperature in celsius.
 * Traw is the raw value read from the device.
 * offset is (at 35 degrees):
 * offset = 35 - (-521/340) = 35 + (521/340)
 * so to calculate the T:
 * T = (Traw/340) + 35 + (521/340)
 *   = (Traw/340) + (35*340/340) + (521/340)
 *   = (Traw + 11900 + 521) / 340
 *   = (Traw + 12421) / 340
 */
uint8_t mpu6050_get_temperature(struct mpu6050_t *mpu6050)
{
	uint8_t err;
	int raw_temp;
	uint16_t uraw = 0;

	/* Read the raw temperature */
	err = register_rw(MPU6050_ADDR, MPU6050_RA_TEMP_OUT_H, &uraw);
	raw_temp = (int)uraw;
	mpu6050->temp = ((float)raw_temp + 12421.0) / 340.0;

	return (err);
}

uint8_t mpu6050_read_all(struct mpu6050_t *mpu6050)
{
	uint8_t err;
	uint16_t word;

	/* Read the accel */
	err = register_rw(MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, &word);
	mpu6050->ax = (int)word;
	err |= register_rw(MPU6050_ADDR, MPU6050_RA_ACCEL_YOUT_H, &word);
	mpu6050->ay = (int)word;
	err |= register_rw(MPU6050_ADDR, MPU6050_RA_ACCEL_ZOUT_H, &word);
	mpu6050->az = (int)word;
	/* Read the gyro */
	err |= register_rw(MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H, &word);
	mpu6050->gx = (int)word;
	err |= register_rw(MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H, &word);
	mpu6050->gy = (int)word;
	err |= register_rw(MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H, &word);
	mpu6050->gz = (int)word;

	err |= mpu6050_get_temperature(mpu6050);

	return (err);
}
