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
 * @param reg_addr the register address.
 * @param byte the data to be read.
 */
uint8_t register_rb(const uint8_t reg_addr, uint8_t *byte)
{
	uint8_t error;

	error = i2c_master_send_b(MPU6050_ADDR, reg_addr, FALSE);

	if (!error)
		error = i2c_master_read_b(MPU6050_ADDR, byte, TRUE);

	return (error);
}

/** Register Read (word).
 *
 * @param reg_addr the register address.
 * @param byte the data to be read.
 */
uint8_t register_rw(const uint8_t reg_addr, uint16_t *word)
{
	uint8_t err;

	err = i2c_master_send_b(MPU6050_ADDR, reg_addr, FALSE);

	if (!err)
		err = i2c_master_read_w(MPU6050_ADDR, word);

	/*
		err = i2c_master_read_w(MPU6050_ADDR, word, TRUE);
		*/

	return (err);
}

/** Register write (Byte).
 *
 * @param reg_addr the register address.
 * @param byte the data to be written.
 */
uint8_t register_wb(const uint8_t reg_addr, uint8_t byte)
{
	uint8_t error;

	error = i2c_master_send_w(MPU6050_ADDR, reg_addr, byte);
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
uint8_t mpu6050_read_temperature(struct mpu6050_t *mpu6050)
{
	uint8_t err;
	int raw_temp;
	uint16_t uraw;

	/* Read the raw temperature */
	err = register_rw(MPU6050_RA_TEMP_OUT, &uraw);
	raw_temp = (int)uraw;
	mpu6050->temp = ((float)raw_temp + 12421.0) / 340.0;

	return (err);
}

uint8_t mpu6050_read_accel(struct mpu6050_t *mpu6050)
{
	uint8_t err;
	uint16_t word;

	/* Read the accel */
	err = register_rw(MPU6050_RA_ACCEL_XOUT, &word);
	mpu6050->ax = (int)word;
	err |= register_rw(MPU6050_RA_ACCEL_YOUT, &word);
	mpu6050->ay = (int)word;
	err |= register_rw(MPU6050_RA_ACCEL_ZOUT, &word);
	mpu6050->az = (int)word;

	return (err);
}

/** Read the gyro.
 *
 */
uint8_t mpu6050_read_gyro(struct mpu6050_t *mpu6050)
{
	uint8_t err;
	uint16_t word;

	err = register_rw(MPU6050_RA_GYRO_XOUT, &word);
	mpu6050->gx = (int)word - mpu6050->offset_gx;
	err |= register_rw(MPU6050_RA_GYRO_YOUT, &word);
	mpu6050->gy = (int)word - mpu6050->offset_gy;
	err |= register_rw(MPU6050_RA_GYRO_ZOUT, &word);
	mpu6050->gz = (int)word - mpu6050->offset_gz;

	return (err);
}

/** Calibrate the gyroscope.
 *
 * Perform 255 readings and mediate them in order to
 * get the offsets.
 * This has to be done in a stable situation.
 * The sum of readings of any of the axis has to be stored
 * in an INT or the calibration fail!
 *
 * @note use the accelerometer to check the stability during
 * the calibration?
 * @bug Calibration drift should be small numbers.
 */
uint8_t mpu6050_gyro_calibrate(struct mpu6050_t *mpu6050)
{
	uint8_t i, err;
	int sx, sy, sz;

	err = 0;
	sx = 0;
	sy = 0;
	sz = 0;
	mpu6050->offset_gx = 0;
	mpu6050->offset_gy = 0;
	mpu6050->offset_gz = 0;

	for (i = 0; i < MPU6050_GYRO_CALIBRATION; i++) {
		err |= mpu6050_read_gyro(mpu6050);
		sx += mpu6050->gx;
		sy += mpu6050->gy;
		sz += mpu6050->gz;
	}

	mpu6050->offset_gx = (int)(sx/MPU6050_GYRO_CALIBRATION);
	mpu6050->offset_gy = (int)(sy/MPU6050_GYRO_CALIBRATION);
	mpu6050->offset_gz = (int)(sz/MPU6050_GYRO_CALIBRATION);

	if (!err)
		mpu6050->flags |= _BV(MPU6050_FLAG_CALIBRATED);

	return (err);
}

/** Init
 * The default id is 0b110100
 */
uint8_t mpu6050_init(struct mpu6050_t *mpu6050)
{
	uint8_t byte, err;

	mpu6050->flags = 0;

	i2c_init();

	err = register_rb(MPU6050_RA_WHO_AM_I, &byte);

	if (err || (byte != 0x68)) {
		err |= 0x80; /* give me a name error */
		mpu6050->flags |= _BV(MPU6050_FLAG_COM_ERR);
	}

	if (!err) {
		/* Sample rate /8 */
		err = register_wb(MPU6050_RA_SMPLRT_DIV, 7);
		err |= register_wb(MPU6050_RA_CONFIG, 6);
		/* Full scale */
		err |= register_wb(MPU6050_RA_GYRO_CONFIG, 0x18);
		/* Accel Range +-2G, HPF 1 */
		err |= register_wb(MPU6050_RA_ACCEL_CONFIG, 1);
		/* No sleep
		 * clksrc Internal 8KHz
		 * Temp. enable.
		 */
		err |= register_wb(MPU6050_RA_PWR_MGMT_1, 1);
	} else {
		mpu6050->flags |= _BV(MPU6050_FLAG_COM_ERR);
	}

	if (!err)
		err = mpu6050_gyro_calibrate(mpu6050);
	else
		mpu6050->flags |= _BV(MPU6050_FLAG_COM_ERR);

	return (err);
}

uint8_t mpu6050_read_all(struct mpu6050_t *mpu6050)
{
	uint8_t err;

	err = mpu6050_read_accel(mpu6050);
	err |= mpu6050_read_gyro(mpu6050);
	err |= mpu6050_read_temperature(mpu6050);

	return (err);
}

/** Configure Low Power Accel for Motion Interrupt (LPA).
 *
 * @bug Make sure Accel is running.
 */
uint8_t lpa_start(struct mpu6050_t *mpu6050)
{
	uint8_t err, reg;

	/* Step 1
	 * In PWR_MGMT_1 make cycle=0 and sleep=0
	 * In PWR_MGMT_2 make STBY_[XYZ]A=0
	 */
	err = register_rb(MPU6050_RA_PWR_MGMT_1, &reg);

	if (!err) {
		reg &= ~_BV(MPU6050_PWR1_SLEEP_BIT | MPU6050_PWR1_CYCLE_BIT);
		err = register_wb(MPU6050_RA_PWR_MGMT_1, reg);
	}

	err |= register_rb(MPU6050_RA_PWR_MGMT_2, &reg);

	if (!err) {
		/* STBY Accel clear */
		reg &= ~_BV(MPU6050_PWR2_STBY_XA_BIT |
				MPU6050_PWR2_STBY_YA_BIT |
				MPU6050_PWR2_STBY_ZA_BIT);
		err = register_wb(MPU6050_RA_PWR_MGMT_2, reg);
	}

	/* Step 2
	 * Set Accel DHPF to reset settings.
	 */
	err |= register_rb(MPU6050_RA_ACCEL_CONFIG, &reg);

	if (!err) {
		reg &= 0xF8; /* 0b11111000 */
		err = register_wb(MPU6050_RA_ACCEL_CONFIG, reg);
	}

	/* Step 3
	 * Set Accel DLPF.
	 */
	err |= register_rb(MPU6050_RA_CONFIG, &reg);

	if (!err) {
		reg &= 0xF8; /* 0b11111000 */
		err = register_wb(MPU6050_RA_CONFIG, reg);
	}

	/* Step pre-4
	 * Set IRQ active low.
	 * unused.
	 */
#ifdef MPU6050_IRQ_LINE_LOW
	err |= register_rb(MPU6050_RA_INT_PIN_CFG, &reg);

	if (!err) {
		* set INT_LEVEL (bit 7) to 1 *
		reg |= _BV(7);
		err = register_wb(MPU6050_RA_INT_PIN_CFG, reg);
	}
#endif

	/* Step 4
	 * Set motion IRQ to enable.
	 */
	if (!err) {
		err = register_wb(MPU6050_RA_INT_ENABLE, 0x40);
	}

	/* Step 5
	 * Set motion duration.
	 */
	if (!err) {
		err = register_wb(MPU6050_RA_MOT_DUR, MPU6050_LPA_MOT_DUR);
	}

	/* Step 6
	 * Set motion threshold.
	 */
	if (!err) {
		err = register_wb(MPU6050_RA_MOT_THR, MPU6050_LPA_MOT_THR);
	}

	/* Step 6 - alien
	 * Set Non valid detect control.
	 */
	/*
	if (!err) {
		err = register_wb(MPU6050_RA_MOT_DETECT_CTRL,
			       MPU6050_LPA_DETECT_CTRL);
	}
	*/

	/* Step 7
	 * Add delay. Give at least 1ms delay for accumulating
	 * samples.
	 */
	_delay_ms(4);

	/* Step 8
	 * Set Accel HPF to HOLD settings.
	 */
	err |= register_rb(MPU6050_RA_ACCEL_CONFIG, &reg);

	if (!err) {
		/* set HPF to 0b111 */
		reg |= 7;
		err = register_wb(MPU6050_RA_ACCEL_CONFIG, reg);
	}

	/* Step 9
	 * Set the frequency of wakeup and put the gyro in stdby.
	 */
	err |= register_rb(MPU6050_RA_PWR_MGMT_2, &reg);

	if (!err) {
		reg &= 0x38; /* 0b00111000 */
		/* stdby for gyro */
		reg |= _BV(2 | 1 | 0);
		reg |= (MPU6050_LPA_WAKE_CTRL << 6);
		err = register_wb(MPU6050_RA_PWR_MGMT_2, reg);
	}

	/* Step 10
	 * Enable cycle mode (Low Power Mode).
	 */
	err |= register_rb(MPU6050_RA_PWR_MGMT_1, &reg);

	if (!err) {
		reg |= _BV(5);
		reg &= ~_BV(6);
		err = register_wb(MPU6050_RA_PWR_MGMT_1, reg);
	}

	return (err);
}

uint8_t lpa_restart(struct mpu6050_t *mpu6050)
{
	uint8_t err, reg;

	/* Step 4
	 * Set motion IRQ to enable.
	 */
	err = register_wb(MPU6050_RA_INT_ENABLE, 0x40);

	/* Step 10
	 * Enable cycle mode (Low Power Mode).
	 */
	err |= register_rb(MPU6050_RA_PWR_MGMT_1, &reg);

	if (!err) {
		reg |= _BV(5);
		reg &= ~_BV(6);
		err = register_wb(MPU6050_RA_PWR_MGMT_1, reg);
	}

	return (err);
}

uint8_t lpa_suspend(struct mpu6050_t *mpu6050)
{
	uint8_t err, reg;

	/* Step 10
	 * Disable cycle mode (Low Power Mode).
	 */
	err = register_rb(MPU6050_RA_PWR_MGMT_1, &reg);

	if (!err) {
		reg &= 0x9F; /* 0b10011111 */
		err = register_wb(MPU6050_RA_PWR_MGMT_1, reg);
	}

	 /* Disable motion IRQ */
	if (!err) {
		err = register_wb(MPU6050_RA_INT_ENABLE, 0);
	}

	return(err);
}

uint8_t lpa_stop(struct mpu6050_t *mpu6050)
{
	return(0);
}

uint8_t mpu6050_LPA(uint8_t mode, struct mpu6050_t *mpu6050)
{
	switch (mode) {
		case LPA_START:
			lpa_start(mpu6050);
			break;
		case LPA_RESTART:
			lpa_restart(mpu6050);
			break;
		case LPA_SUSPEND:
			lpa_suspend(mpu6050);
			break;
		case LPA_STOP:
		default:
			lpa_stop(mpu6050);
	}

	return (0);
}

uint8_t mpu6050_read_irq(uint8_t *byte) {
	return(register_rb(MPU6050_RA_INT_STATUS, byte));
}

uint8_t mpu6050_read_mot_detect_status(uint8_t *byte) {
	return(register_rb(MPU6050_RA_MOT_DETECT_STATUS, byte));
}
