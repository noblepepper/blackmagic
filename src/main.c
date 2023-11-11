/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Provides main entry point. Initialise subsystems */

#include "general.h"
#include "platform.h"
#include "usbwrap.h"
#include "setup.h"
#include <libopencm3/stm32/usart.h>

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	clock_setup();
	systick_setup();
	usb_setup();
	usart_setup();
	gpio_setup();

	while (true) {
		if (usb_data_waiting())
			usart_send_blocking(USBUSART, usb_recv_blocking());
		if (usart_data_waiting(USBUSART))
		{
			usb_send_blocking(usart_recv(USBUSART));
		}
		asm("nop");
	}

	return 0;
}
