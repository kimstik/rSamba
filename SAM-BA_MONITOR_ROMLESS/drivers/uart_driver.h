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


#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "device_config.h"

#if SAM_BA_UART_INTERFACE_ENABLED

/**
 * \brief Initializes the UART
 *
 * \param Pointer to SERCOM instance
 * \param Baud value corresponding to the desired baudrate
 * \param SERCOM pad settings
 */
void uart_basic_init(Sercom *sercom, uint16_t baud_val, uint32_t pad_conf);

/**
 * \brief Disables UART interface
 *
 * \param Pointer to SERCOM instance
 */
void uart_disable(Sercom *sercom);

/**
 * \brief Sends a single byte through UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Data to send
 */
void uart_write_byte(Sercom *sercom, uint8_t data);

/**
 * \brief Reads a single character from UART interface
 *
 * \param Pointer to SERCOM instance
 * \return Data byte read
 */
uint8_t uart_read_byte(Sercom *sercom);

/**
 * \brief Sends buffer on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to data to send
 * \param Number of bytes to send
 */
void uart_write_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);

/**
 * \brief Reads data on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to store read data
 * \param Number of bytes to read
 */
void uart_read_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);
#endif
#endif