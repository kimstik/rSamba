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


#ifndef _USART_SAM_BA_H_
#define _USART_SAM_BA_H_

#include <stdint.h>
#include <stdbool.h>


/* Define the default time-out value for USART. */
#define USART_DEFAULT_TIMEOUT    1000

/* Xmodem related defines */
/* CRC16  polynomial */
#define CRC16POLY                0x1021

#define SHARP_CHARACTER          '#'

/* X/Ymodem protocol: */
#define SOH                      0x01
//#define STX                    0x02
#define EOT                      0x04
#define ACK                      0x06
#define NAK                      0x15
#define CAN                      0x18
#define ESC                      0x1b

#define PKTLEN_128               128


/**
 * \brief Open the given USART
 */
void usart_open(void);

/**
 * \brief Stops the USART
 */
void usart_close(void);

/**
 * \brief Puts a byte on usart line
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int usart_putc(int value);

/**
 * \brief Waits and gets a value on usart line
 *
 * \return value read on usart line
 */
int usart_getc(void);

/**
 * \brief Returns true if the SAM-BA Uart received the sharp char
 *
 * \return Returns true if the SAM-BA Uart received the sharp char
 */
int usart_sharp_received(void);

/**
 * \brief This function checks if a character has been received on the usart line
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool usart_is_rx_ready(void);

/**
 * \brief Send buffer on usart line
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t usart_putdata(void const* data, uint32_t length); //Send given data (polling)

/**
 * \brief Gets data from usart line
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t usart_getdata(void* data, uint32_t length); //Get data from comm. device

/**
 * \brief Send buffer on usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t usart_putdata_xmd(void const* data, uint32_t length); //Send given data (polling) using xmodem (if necessary)

/**
 * \brief Gets data from usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t usart_getdata_xmd(void* data, uint32_t length); //Get data from comm. device using xmodem (if necessary)

/**
 * \brief Gets data from usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
unsigned short add_crc(char ptr, unsigned short crc);

uint8_t getPacket(uint8_t *pData, uint8_t sno);

#endif // _USART_SAM_BA_H_
