/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>

#include "general.h"
#include "usb.h"
// #include "cdcacm.h"
#include "usbwrap.h"

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

/* USB incoming buffer */
char buf_usb_in[256];
int usb_data_count;

bool usb_data_waiting(void)
{
	return ( usb_data_count != 0);
}

bool usart_data_waiting(uint32_t port)
{
	return ((USART_SR(port) & USART_SR_RXNE) != 0);
}

void usb_wait_send_ready()
{
}

void usb_send(uint16_t data)
{
	uint8_t packet_buf[CDCACM_PACKET_SIZE];
	uint8_t packet_size = 0;
	packet_buf[0]=data;
	packet_size = 1;
	usbd_ep_write_packet(usbdev, 3, packet_buf, packet_size);
}

void usb_send_blocking(uint16_t data)
{
	usb_wait_send_ready();
	usb_send(data);
}

/* incoming data from usb host to us */
void read_from_usb(usbd_device *dev, uint8_t ep)
{
	(void)ep;
	int i;

	char buf[CDCACM_PACKET_SIZE];
	int len = usbd_ep_read_packet(dev, 3,
					buf, CDCACM_PACKET_SIZE);

	//gpio_set(LED_PORT_UART, LED_UART);
	for(i = 0; i < len; i++){
		buf_usb_in[i] = buf[i];
		usb_data_count++;
	}
	//gpio_clear(LED_PORT_UART, LED_UART);

}

void usb_wait_recv_ready(void)
{
	while (usb_data_count == 0);
}

uint16_t usb_recv(void)
{
	usb_data_count--;
	return buf_usb_in[0];
}

uint16_t usb_recv_blocking(void)
{
	usb_wait_recv_ready();
	return usb_recv();
}


/* sends data out from us to the usb host*/
void send_to_usb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
}
