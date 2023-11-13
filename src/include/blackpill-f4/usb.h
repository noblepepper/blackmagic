
#ifndef BLACKPILL_F4_USB_H
#define BLACKPILL_F4_USB_H

#include "gpio.h"

#define USB_DRIVER stm32f107_usb_driver
#define USB_IRQ    NVIC_OTG_FS_IRQ
#define IRQ_PRI_USB          (1U << 4U)

#define UART_PIN_SETUP()                                                                            \
	do {                                                                                            \
		gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);              \
		gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO2); \
		gpio_set_af(GPIOA, GPIO_AF7, GPIO2);                                      \
		gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3);            \
		gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO3); \
		gpio_set_af(GPIOA, GPIO_AF7, GPIO3);                                      \
	} while (0)

#endif /* BLACKPILL_F4_USB_H */
