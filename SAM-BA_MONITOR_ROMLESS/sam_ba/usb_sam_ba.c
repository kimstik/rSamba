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


#include "cdc_enumerate.h"
#include "usb_sam_ba.h"
#include "device_config.h"

#if SAM_BA_USB_INTERFACE_ENABLED

extern USB_CDC pCdc;

void usb_init(void)
{
	pCdc.pUsb = USB;
	/* Initialize USB */
	AT91F_InitUSB();
	/* Get the default CDC structure settings */
	AT91F_CDC_Open((P_USB_CDC)&pCdc, pCdc.pUsb);
}

int cdc_putc(int value)
{
	/* Send single byte on USB CDC */
	USB_Write(&pCdc, (const char *)&value, 1, USB_EP_IN);
	return 1;
}

int cdc_getc(void)
{
	uint8_t rx_char;
	/* Read singly byte on USB CDC */
	USB_Read(&pCdc, (char *)&rx_char, 1);
	return (int)rx_char;
}

bool cdc_is_rx_ready(void)
{
	/* Check whether the device is configured */
	if ( !USB_IsConfigured(&pCdc))
		return 0;

	/* Return transfer complete 0 flag status */
	return (pCdc.pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0);
}

uint32_t cdc_write_buf(void const* data, uint32_t length)
{
	/* Send the specified number of bytes on USB CDC */
	USB_Write(&pCdc, (const char *)data, length, USB_EP_IN);
	return length;
}

uint32_t cdc_read_buf(void* data, uint32_t length)
{
	/* Check whether the device is configured */
	if ( !USB_IsConfigured(&pCdc))
		return 0;

	/* Read from USB CDC */
	return USB_Read(&pCdc, (char *)data, length);
}

uint32_t cdc_read_buf_xmd(void* data, uint32_t length)
{
	/* Check whether the device is configured */
	if ( !USB_IsConfigured(&pCdc))
		return 0;

	/* Blocking read till specified number of bytes is received */
	return USB_Read_blocking(&pCdc, (char *)data, length);
}
#endif
