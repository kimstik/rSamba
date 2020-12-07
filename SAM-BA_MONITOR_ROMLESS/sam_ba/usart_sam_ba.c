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



#include "usart_sam_ba.h"
#include "uart_driver.h"

#if SAM_BA_UART_INTERFACE_ENABLED

/* Test for timeout in AT91F_GetChar */
uint16_t size_of_data;
uint8_t mode_of_transfer;


/**
 * \brief Configures communication line
 *
 */
void usart_close(void)
{
	uart_disable(BOOT_USART_MODULE);
}

/**
 * \brief Puts a byte on usart line
 * The type int is used to support printf redirection from compiler LIB.
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int usart_putc(int value)
{
	uart_write_byte(BOOT_USART_MODULE, (uint8_t)value);
	return 1;
}



int usart_getc(void) {
	uint16_t retval;
	//Wait until input buffer is filled
	while(!(usart_is_rx_ready()));
	retval = (uint16_t)uart_read_byte(BOOT_USART_MODULE);
	//usart_read_wait(&usart_sam_ba, &retval);
	return (int)retval;

}

int usart_sharp_received(void) {
	if (usart_is_rx_ready()) {
		if (usart_getc() == SHARP_CHARACTER)
			return (true);
	}
	return (false);
}

//Send given data (polling)
uint32_t usart_putdata(void const* data, uint32_t length) {
	uint32_t i;
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	for (i = 0; i < length; i++) {
		usart_putc(*ptrdata);
		ptrdata++;
	}
	return (i);
}

//Get data from comm. device
uint32_t usart_getdata(void* data, uint32_t length) {
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	*ptrdata = usart_getc();
	return (1);
}

//*----------------------------------------------------------------------------
//* \fn    add_crc
//* \brief Compute the CRC
//*----------------------------------------------------------------------------
unsigned short add_crc(char ptr, unsigned short crc) {

	unsigned short cmpt;

	crc = crc ^ (int) ptr << 8;

	for (cmpt = 0; cmpt < 8; cmpt++) {
		if (crc & 0x8000)
			crc = crc << 1 ^ CRC16POLY;
		else
			crc = crc << 1;
	}

	return (crc & 0xFFFF);
}

//*----------------------------------------------------------------------------
//* \fn    getbytes
//* \brief
//*----------------------------------------------------------------------------
static uint16_t getbytes(uint8_t *ptr_data, uint16_t length) {
	uint16_t crc = 0;
	uint16_t cpt;
	uint8_t c;

	for (cpt = 0; cpt < length; ++cpt) {
		c = usart_getc();
		crc = add_crc(c, crc);
		//crc = (crc << 8) ^ xcrc16tab[(crc>>8) ^ c];
		if (size_of_data || mode_of_transfer) {
			*ptr_data++ = c;
			if (length == PKTLEN_128)
				size_of_data--;
		}
	}

	return crc;
}

//*----------------------------------------------------------------------------
//* \fn    putPacket
//* \brief Used by Xup to send packets.
//*----------------------------------------------------------------------------
static int putPacket(uint8_t *tmppkt, uint8_t sno) {
	uint32_t i;
	uint16_t chksm;
	uint8_t data;

	chksm = 0;

	usart_putc(SOH);

	usart_putc(sno);
	usart_putc((uint8_t) ~(sno));

	for (i = 0; i < PKTLEN_128; i++) {
		if (size_of_data || mode_of_transfer) {
			data = *tmppkt++;
			size_of_data--;
		} else
			data = 0x00;

		usart_putc(data);

		//chksm = (chksm<<8) ^ xcrc16tab[(chksm>>8)^data];
		chksm = add_crc(data, chksm);
	}

	/* An "endian independent way to extract the CRC bytes. */
	usart_putc((uint8_t) (chksm >> 8));
	usart_putc((uint8_t) chksm);

	return (usart_getc()); /* Wait for ack */
}

//*----------------------------------------------------------------------------
//* \fn    getPacket
//* \brief Used by Xdown to retrieve packets.
//*----------------------------------------------------------------------------
uint8_t getPacket(uint8_t *ptr_data, uint8_t sno) {
	uint8_t seq[2];
	uint16_t crc, xcrc;

	getbytes(seq, 2);
	xcrc = getbytes(ptr_data, PKTLEN_128);

	/* An "endian independent way to combine the CRC bytes. */
	crc = (uint16_t) usart_getc() << 8;
	crc += (uint16_t) usart_getc();

	if ((crc != xcrc) || (seq[0] != sno) || (seq[1] != (uint8_t) (~sno))) {
		usart_putc(CAN);
		return (false);
	}

	usart_putc(ACK);
	return (true);
}

//*----------------------------------------------------------------------------
//* \fn    Xup
//* \brief Called when a transfer from target to host is being made (considered
//*        an upload).
//*----------------------------------------------------------------------------
//static void Xup(char *ptr_data, uint16_t length)
//Send given data (polling) using xmodem (if necessary)
uint32_t usart_putdata_xmd(void const* data, uint32_t length) {
	uint8_t c, sno = 1;
	uint8_t done;
	uint8_t * ptr_data = (uint8_t *) data;

	if (!length)
		mode_of_transfer = 1;
	else {
		size_of_data = length;
		mode_of_transfer = 0;
	}

	if (length & (PKTLEN_128 - 1)) {
		length += PKTLEN_128;
		length &= ~(PKTLEN_128 - 1);
	}

	/* Startup synchronization... */
	/* Wait to receive a NAK or 'C' from receiver. */
	done = 0;
	while (!done) {
		c = (uint8_t) usart_getc();

		switch (c) {
			case NAK:
				done = 1;
				// ("CSM");
				break;
			case 'C':
				done = 1;
				// ("CRC");
				break;
			case 'q': /* ELS addition, not part of XMODEM spec. */
				return (0);
			default:
				break;
		}
	}

	done = 0;
	sno = 1;
	while (!done) {
		c = (uint8_t) putPacket((uint8_t *) ptr_data, sno);

		switch (c) {
			case ACK:
				++sno;
				length -= PKTLEN_128;
				ptr_data += PKTLEN_128;
				// ("A");
				break;
			case NAK:
				// ("N");
				break;
			case CAN:
			case EOT:
			default:
				done = 0;
				break;
		}
		if (!length) {
			usart_putc(EOT);
			usart_getc(); /* Flush the ACK */
			break;
		}
		// ("!");
	}

	mode_of_transfer = 0;
	// ("Xup_done.");
	return (1);
	//    return(0);
}

//*----------------------------------------------------------------------------
//* \fn    Xdown
//* \brief Called when a transfer from host to target is being made (considered
//*        an download).
//*----------------------------------------------------------------------------
//static void Xdown(char *ptr_data, uint16_t length)
//Get data from comm. device using xmodem (if necessary)
uint32_t usart_getdata_xmd(void* data, uint32_t length) {
	uint32_t timeout;
	char c;
	uint8_t * ptr_data = (uint8_t *) data;
	uint32_t b_run, nbr_of_timeout = 100;
	uint8_t sno = 0x01;
	uint32_t data_transfered = 0;

	//Copied from legacy source code ... might need some tweaking
	uint32_t loops_per_second = CPU_FREQUENCY/10; /* system_clock_source_get_hz(BOOT_USART_GCLK_GEN_SOURCE) / 10; */

	if (length == 0)
		mode_of_transfer = 1;
	else {
		size_of_data = length;
		mode_of_transfer = 0;
	}

	/* Startup synchronization... */
	/* Continuously send NAK or 'C' until sender responds. */
	// ("Xdown");
	while (1) {
		usart_putc('C');
		timeout = loops_per_second;
		while (!(usart_is_rx_ready()) && timeout)
			timeout--;
		if (timeout)
			break;

		if (!(--nbr_of_timeout))
			return (0);
//            return -1;
	}

	b_run = true;
	// ("Got response");
	while (b_run != false) {
		c = (char) usart_getc();

		switch (c) {
			case SOH: /* 128-byte incoming packet */
				// ("O");
				b_run = getPacket(ptr_data, sno);
				if (b_run == true) {
					++sno;
					ptr_data += PKTLEN_128;
					data_transfered += PKTLEN_128;
				}
				break;
			case EOT: // ("E");
				usart_putc(ACK);
				b_run = false;
				break;
			case CAN: // ("C");
			case ESC: /* "X" User-invoked abort */
			default:
				b_run = false;
				break;
		}
		// ("!");
	}
	mode_of_transfer = 0;
	return (true);
//    return(b_run);
}
#endif
