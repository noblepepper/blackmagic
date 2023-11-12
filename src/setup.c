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

/* This file implements the platform specific functions for the blackpill-f4 implementation. */

#include "general.h"
#include "platform.h"
#include "usb.h"
#include "aux_serial.h"
#include "serialno.h"
#include "usb_descriptors.h"
#include "usb_serial.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/usb/dwc/otg_fs.h>


#define SYSTICKHZ 1000U

#define SYSTICKMS (1000U / SYSTICKHZ)

static volatile uint32_t time_ms = 0;

void clock_setup(void)
{
	/* main clock 96Mhz */
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[PLATFORM_CLOCK_FREQ]);

	/* Enable GPIO peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable usb peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
}

void systick_setup(void)
{
	/* Setup heartbeat timer */
	/* 12 Mhz */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* Interrupt us at 100 Hz */
	systick_set_reload(rcc_ahb_frequency / (8U * SYSTICKHZ));
	/* SYSTICK_IRQ with low priority */
	nvic_set_priority(NVIC_SYSTICK_IRQ, 14U << 4U);
	systick_interrupt_enable();
	systick_counter_enable();
}

void sys_tick_handler(void)
{
	time_ms += SYSTICKMS;
}

void usart_setup(void)
{
	/* Enable clocks */
	rcc_periph_clock_enable(USBUSART_CLK);
	rcc_periph_clock_enable(USBUSART_DMA_CLK);

	/* Setup UART parameters */
	UART_PIN_SETUP();
	usart_set_baudrate(USBUSART, 115200);
	usart_set_databits(USBUSART, 8);
	usart_set_stopbits(USBUSART, USART_STOPBITS_1);
	usart_set_mode(USBUSART, USART_MODE_TX_RX);
	usart_set_parity(USBUSART, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART, USART_FLOWCONTROL_NONE);
	USART_CR1(USBUSART) |= USART_CR1_IDLEIE;

	/* Finally enable the USART */
	usart_enable(USBUSART);
	usart_enable_tx_dma(USBUSART);
	usart_enable_rx_dma(USBUSART);
}

void gpio_setup(void)
{
	/* Set up LED pins */
}

static bool usb_config_updated = true;

static void usb_config_set_updated(usbd_device *const dev, const uint16_t value)
{
	(void)dev;
	(void)value;
	usb_config_updated = true;
}

void usb_setup(void)
{
/* We need a special large control buffer for this device: */
static uint8_t usbd_control_buffer[512];

	/* Set up DM/DP pins. PA9/PA10 are not routed to USB-C. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOA_OSPEEDR &= 0x3c00000cU;
	GPIOA_OSPEEDR |= 0x28000008U;
//	read_serial_number();

	usbdev = usbd_init(&USB_DRIVER, &dev_desc, &config, usb_strings,
		 ARRAY_LENGTH(usb_strings), usbd_control_buffer,
		 sizeof(usbd_control_buffer));

	usbd_register_bos_descriptor(usbdev, &bos);
	usbd_register_set_config_callback(usbdev, usb_serial_set_config);
	usbd_register_set_config_callback(usbdev, usb_config_set_updated);

	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
	
	/* https://github.com/libopencm3/libopencm3/pull/1256#issuecomment-779424001 */
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);

}
