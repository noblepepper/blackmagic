/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
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
#include "general.h"
#include "platform.h"
#include <libopencm3/stm32/desig.h>

char serial_no[9];

void read_serial_number(void)
{
	const volatile uint32_t *const unique_id_p = (uint32_t *)DESIG_UNIQUE_ID_BASE;
	const uint32_t unique_id = unique_id_p[0] + unique_id_p[1] + unique_id_p[2];
	/* Fetch serial number from chip's unique ID */
	for (size_t i = 0; i < 8U; ++i) {
		serial_no[7U - i] = ((unique_id >> (i * 4U)) & 0x0fU) + '0';
		/* If the character is something above 9, then add the offset to make it ASCII A-F */
		if (serial_no[7U - i] > '9')
			serial_no[7U - i] += 7; /* 'A' - '9' = 8, less 1 gives 7. */
	}
	serial_no[8] = '\0';
}
