/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/


#include "uart_driver.h"
#include "device_config.h"

#if SAM_BA_UART_INTERFACE_ENABLED
/**
 * \brief Open the given USART
 */
void usart_open()
{
	configure_sercom_port_pins();
	clock_configuration_for_boot_usart(); 

	uart_basic_init(BOOT_USART_MODULE, USART_BAUD_REG_VAL_FOR_SAMBA, BOOT_USART_MUX_SETTINGS);
}

void uart_basic_init(Sercom *sercom, uint16_t baud_val, uint32_t pad_conf)
{
	/* Wait for synchronization */
	wait_for_usart_enable_sync(sercom);
	/* Disable the SERCOM UART module */
	sercom->USART.CTRLA.bit.ENABLE = 0;
	/* Wait for synchronization */
	wait_for_usart_swrst_sync(sercom);
	/* Perform a software reset */
	sercom->USART.CTRLA.bit.SWRST = 1;
	/* Wait for synchronization */
	wait_for_usart_swrst_sync(sercom);
	/* Wait for synchronization */
	wait_for_usart_swrst_enable_sync(sercom);
	/* Update the UART pad settings, mode and data order settings */
	sercom->USART.CTRLA.reg = pad_conf | SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_RUNSTDBY;
	/* Wait for synchronization */
	wait_for_usart_ctrlb_sync(sercom);
	/* Enable transmit and receive and set data size to 8 bits */
	sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	/* Load the baud value */
	sercom->USART.BAUD.reg = baud_val;
	/* Wait for synchronization */
	wait_for_usart_enable_sync(sercom);
	/* Enable SERCOM UART */
	sercom->USART.CTRLA.bit.ENABLE = 1;
}

void uart_disable(Sercom *sercom)
{
	/* Wait for synchronization */
	wait_for_usart_enable_sync(sercom);
	/* Disable SERCOM UART */
	sercom->USART.CTRLA.bit.ENABLE = 0;
}

void uart_write_byte(Sercom *sercom, uint8_t data)
{
	wait_for_uart_syncbusy_clear(sercom);
	
	/* Write the data to DATA register */
	sercom->USART.DATA.reg = (uint16_t)data;

	while (!(sercom->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC));
}

uint8_t uart_read_byte(Sercom *sercom)
{
	wait_for_uart_syncbusy_clear(sercom);
	uart_read_clear_errors(sercom);
	
	/* Return the read data */
	return((uint8_t)sercom->USART.DATA.reg);
}

void uart_write_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length)
{
	/* Do the following for specified length */
	do {
		/* Wait for Data Register Empty flag */
		while(!sercom->USART.INTFLAG.bit.DRE);
		/* Send data from the buffer */
		sercom->USART.DATA.reg = (uint16_t)*ptr++;
	} while (length--);
}

void uart_read_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length)
{
	/* Do the following for specified length */
	do {
		/* Wait for Receive Complete flag */
		while(!sercom->USART.INTFLAG.bit.RXC);
		uart_read_clear_errors(sercom);

		/* Store the read data to the buffer */
		*ptr++ = (uint8_t)sercom->USART.DATA.reg;
	} while (length--);
}

bool usart_is_rx_ready(void) {
	return (BOOT_USART_MODULE->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC);
}
#endif
