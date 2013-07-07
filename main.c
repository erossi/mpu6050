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
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "mpu6050.h"
#include "uart.h"

/* Global IRQ volatile flag */
volatile uint8_t irq_flag;

/*! IRQ wakes up when chip send an IRQ. */
ISR(INT0_vect)
{
	irq_flag = 1;
}

/*! enable irq int0 */
void irq_ena(void)
{
        /* PD2 input */

        /* Trigger irq on rising edge of the INT0 pin */
        EICRA |= _BV(ISC01 | ISC00);
        /* Enable irq0 */
        EIMSK |= _BV(INT0);
}

/*! disable INT0 irq */
void irq_shut(void)
{
        /* disable irq0 */
        EIMSK &= ~_BV(INT0);
        /* disable triggers */
        EICRA &= ~(_BV(ISC01) | _BV(ISC00));
}

void print_all(struct mpu6050_t *mpu6050, char *string)
{
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

void print_error(uint8_t error, char *string)
{
	string = utoa(error, string, 16);
	uart_printstr(0, "\nError: ");
	uart_printstr(0, string);
	uart_printstr(0, "\n");
}

void print_irq(char *string)
{
	uint8_t err, irq_reg;

	err = mpu6050_read_irq(&irq_reg);

	if (err) {
		print_error(err, string);
	} else {
		string = utoa(irq_reg, string, 16);
		uart_printstr(0, "IRQ reg: 0x");
		uart_printstr(0, string);
		uart_printstr(0, "\n");
	}
}

int main(void)
{
	struct mpu6050_t *mpu6050;
	char *string;
	uint8_t lpa, err;
	int i;

	string = malloc(80);
	mpu6050 = malloc(sizeof(struct mpu6050_t));

	irq_flag = 0;
	irq_ena();

	uart_init(0);
	uart_printstr(0, "begin\n");

	_delay_ms(1000);

	err = mpu6050_init(mpu6050);
	string = utoa(err, string, 16);
	uart_printstr(0, "\nInit: 0x");
	uart_printstr(0, string);
	uart_printstr(0, "\n");

	lpa = 0;
	uart_printstr(0, "\nLPA start: ");
	mpu6050_LPA(START, mpu6050);
	uart_printstr(0, "\nLPA started.");

	sei();
	i = 0;

	while(1) {
		if (irq_flag) {
			string = utoa(i, string, 10);
			uart_printstr(0, string);
			uart_printstr(0, " ");
			print_irq(string);
			irq_flag = 0;
			i++;
			_delay_ms(1000);
		}
	}

	free(string);
	return(0);
}
