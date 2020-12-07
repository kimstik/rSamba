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


#ifndef CDC_ENUMERATE_H
#define CDC_ENUMERATE_H

#include "device_config.h"

#if SAM_BA_USB_INTERFACE_ENABLED

#define USB_EP_OUT              2
#define USB_EP_OUT_SIZE         0x40
#define USB_EP_IN               1
#define USB_EP_IN_SIZE          0x40
#define USB_EP_COMM             3
#define MAX_EP                    4

typedef struct _USB_CDC
{
	// Private members
	Usb *pUsb;
	uint8_t currentConfiguration;
	uint8_t currentConnection;
	// Public Methods:
	uint8_t (*IsConfigured)(struct _USB_CDC *pCdc);
	uint32_t  (*Write) (struct _USB_CDC *pCdc, const char *pData, uint32_t length, uint8_t ep_num);
	uint32_t  (*Read)  (struct _USB_CDC *pCdc, char *pData, uint32_t length);
} USB_CDC, *P_USB_CDC;

typedef struct {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} usb_cdc_line_coding_t;

void AT91F_InitUSB(void);
P_USB_CDC AT91F_CDC_Open(P_USB_CDC pCdc,Usb *pUsb);
uint8_t USB_IsConfigured(P_USB_CDC pCdc);
uint32_t USB_Read(P_USB_CDC pCdc, char *pData, uint32_t length);
uint32_t USB_Write(P_USB_CDC pCdc, const char *pData, uint32_t length, uint8_t ep_num);
uint32_t USB_Read_blocking(P_USB_CDC pCdc, char *pData, uint32_t length);

#endif
#endif // CDC_ENUMERATE_H
