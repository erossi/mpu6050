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
#include "uart.h"

int main(void)
{
	struct mpu6050_t *mpu6050;
	char *string;
	uint8_t err;

	string = malloc(80);
	mpu6050 = malloc(sizeof(struct mpu6050_t));

	uart_init(0);
	uart_printstr(0, "begin\n");

	_delay_ms(1000);

	err = mpu6050_init(mpu6050);
	string = utoa(err, string, 16);
	uart_printstr(0, "\nInit: 0x");
	uart_printstr(0, string);
	uart_printstr(0, "\n");

	while(1) {
		err = mpu6050_read_all(mpu6050);
		
		if (err) {
			string = utoa(err, string, 16);
			uart_printstr(0, "\nError: ");
			uart_printstr(0, string);
			uart_printstr(0, "\n");
		} else {
			uart_printstr(0, "A(xyz) G(xyz) T: (");
			string = itoa(mpu6050->ax, string, 10);
			uart_printstr(0, string);
			uart_printstr(0, ", ");
			string = itoa(mpu6050->ay, string, 10);
			uart_printstr(0, string);
			uart_printstr(0, ", ");
			string = itoa(mpu6050->az, string, 10);
			uart_printstr(0, string);

			uart_printstr(0, ") (");
			string = itoa(mpu6050->gx, string, 10);
			uart_printstr(0, string);
			uart_printstr(0, ", ");
			string = itoa(mpu6050->gy, string, 10);
			uart_printstr(0, string);
			uart_printstr(0, ", ");
			string = itoa(mpu6050->gz, string, 10);
			uart_printstr(0, string);

			uart_printstr(0, ") ");
			string = dtostrf(mpu6050->temp, 10, 5, string);
			uart_printstr(0, string);
			uart_printstr(0, "\n");
		}
	}

	free(string);
	return(0);
}
