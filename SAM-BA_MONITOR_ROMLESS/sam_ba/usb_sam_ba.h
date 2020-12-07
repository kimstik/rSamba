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


#ifndef _USB_SAM_BA_H_
#define _USB_SAM_BA_H_

#include "device_config.h"

/**
 * \brief Initializes the USB module
 *
 * \return Pointer to the USB CDC structure
 */
void usb_init(void);

/**
 * \brief Sends a single byte through USB CDC
 *
 * \param Data to send
 * \return number of data sent
 */
int cdc_putc(int value);

/**
 * \brief Reads a single byte through USB CDC
 *
 * \return Data read through USB
 */
int cdc_getc(void);

/**
 * \brief Checks if a character has been received on USB CDC
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool cdc_is_rx_ready(void);

/**
 * \brief Sends buffer on USB CDC
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t cdc_write_buf(void const* data, uint32_t length);

/**
 * \brief Gets data on USB CDC
 *
 * \param data pointer
 * \param number of data to read
 * \return number of data read
 */
uint32_t cdc_read_buf(void* data, uint32_t length);

/**
 * \brief Gets specified number of bytes on USB CDC
 *
 * \param data pointer
 * \param number of data to read
 * \return number of data read
 */
uint32_t cdc_read_buf_xmd(void* data, uint32_t length);


#endif // _USB_SAM_BA_H_
