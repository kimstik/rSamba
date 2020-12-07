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


#include "sam_ba_monitor.h"
#include "usart_sam_ba.h"
#include "usb_sam_ba.h"
#include "uart_driver.h"
#include "cdc_enumerate.h"
#include "device_config.h"


#if (SAM_BA_INTERFACE == SAM_BA_UART_ONLY)
#define HND(x)	uart_if.x
#else
#define HND(x)	ptr_monitor_if->x
#endif

static void print_new_line(void);
const char RomBOOT_Version[sizeof(SAM_BA_VERSION)-1] = SAM_BA_VERSION;

/* Provides one common interface to handle both USART and USB-CDC */
typedef struct
{
	/* send one byte of data */
	int (*put_c)(int value);
	/* Get one byte */
	int (*get_c)(void);
	/* Receive buffer not empty */
	bool (*is_rx_ready)(void);
	/* Send given data (polling) */
	uint32_t (*putdata)(void const* data, uint32_t length);
	/* Get data from comm. device */
	uint32_t (*getdata)(void* data, uint32_t length);
	/* Send given data (polling) using xmodem (if necessary) */
	uint32_t (*putdata_xmd)(void const* data, uint32_t length);
	/* Get data from comm. device using xmodem (if necessary) */
	uint32_t (*getdata_xmd)(void* data, uint32_t length);
} t_monitor_if;

#if SAM_BA_UART_INTERFACE_ENABLED
/* Initialize structures with function pointers from supported interfaces */
const t_monitor_if uart_if =
{ usart_putc, usart_getc, usart_is_rx_ready, usart_putdata, usart_getdata,
		usart_putdata_xmd, usart_getdata_xmd };
#endif

#if SAM_BA_USB_INTERFACE_ENABLED
extern USB_CDC pCdc;

//Please note that USB doesn't use Xmodem protocol, since USB already includes flow control and data verification
//Data are simply forwarded without further coding.
const t_monitor_if usbcdc_if =
{ cdc_putc, cdc_getc, cdc_is_rx_ready, cdc_write_buf,
		cdc_read_buf, cdc_write_buf, cdc_read_buf_xmd };
#endif

/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

/* b_terminal_mode mode (ascii) or hex mode */
//volatile bool b_terminal_mode = false;
static const bool b_terminal_mode = false;
volatile uint32_t sp;


static void print_new_line(void)
{
	HND(putdata)("\n\r", 2);
}

void init_sam_ba_monitor_interface(void)
{
	#if SAM_BA_UART_INTERFACE_ENABLED
		usart_open();
	#endif

	#if SAM_BA_USB_INTERFACE_ENABLED
		clock_configuration_for_usb();
		usb_init();
	#endif
}

void process_sam_ba_monitor(void)
{
	#if SAM_BA_USB_INTERFACE_ENABLED
		if (pCdc.IsConfigured(&pCdc) != 0) {
			ptr_monitor_if = (t_monitor_if*) &usbcdc_if;
			sam_ba_monitor_run();
		}
	#endif

	#if (SAM_BA_INTERFACE == SAM_BA_UART_ONLY)
		sam_ba_monitor_run();
		for(;;);
	#elif SAM_BA_UART_INTERFACE_ENABLED
		if(uart_if.is_rx_ready() && (SHARP_CHARACTER == uart_if.get_c())) {
			ptr_monitor_if = (t_monitor_if*) &uart_if;
			sam_ba_monitor_run();
			while(1);
		}
	#endif
}

/**
 * \brief This function allows data rx by USART
 *
 * \param *data  Data pointer
 * \param length Length of the data
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length)
{
	uint8_t temp, buf[12], *data_ascii;
	uint32_t i, int_value;

	if (b_terminal_mode)
	{
		if (length == 4)
			int_value = *(uint32_t *) data;
		else if (length == 2)
			int_value = *(uint16_t *) data;
		else
			int_value = *(uint8_t *) data;

		data_ascii = buf + 2;
		data_ascii += length * 2 - 1;

		for (i = 0; i < length * 2; i++)
		{
			temp = (uint8_t) (int_value & 0xf);

			if (temp <= 0x9)
				*data_ascii = temp | 0x30;
			else
				*data_ascii = temp + 0x37;

			int_value >>= 4;
			data_ascii--;
		}
		buf[0] = '0';
		buf[1] = 'x';
		buf[length * 2 + 2] = '\n';
		buf[length * 2 + 3] = '\r';
		HND(putdata)(buf, length * 2 + 4);
	}
	else
		HND(putdata)(data, length);
	return;
}

__attribute__((__noreturn__))
void call_applet(uint32_t address)
{
	uint32_t app_start_address;

	cpu_irq_disable();

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) address);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) address & SCB_VTOR_TBLOFF_Msk);

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(address + 4);

	/* Jump to application Reset Handler in the application */
	__asm("bx %0"::"r"(app_start_address));
	__builtin_unreachable();
}

/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{
	uint32_t length;
	uint32_t j, command, current_number, u8tmp;
	uint8_t *ptr_data, *ptr, data[SIZEBUFMAX];

	j=0;
	command = 0;

	// Start waiting some cmd
	while (1)
	{
		length = HND(getdata)(data, SIZEBUFMAX);
		ptr = data;
		for (uint32_t i = 0; i < length; i++)
		{
			if (*ptr != 0xff)
			{
				if (*ptr == '#')
				{
/*					if (b_terminal_mode)
					{
						print_new_line();
					}
					if (command == 'S')
					{
						//Check if some data are remaining in the "data" buffer
						if(length>i)
						{
							//Move current indexes to next avail data (currently ptr points to "#")
							ptr++;
							i++;
							//We need to add first the remaining data of the current buffer already read from usb
							//read a maximum of "current_number" bytes
							u8tmp=MIN((length-i),current_number);
							for(j=0;j<u8tmp; j++)
							{
								*ptr_data = *ptr;
								ptr_data++;
								ptr++;
								i++;
							}
						}
						//update i with the data read from the buffer
						i--;
						ptr--;
						//Do we expect more data ?
						if(j<current_number)
							HND(getdata_xmd)(ptr_data, current_number-j);

//						__asm("nop");
					}
*/
/*
					else if (command == 'R')
					{
						HND(putdata_xmd)(ptr_data, current_number);
					}
					else if (command == 'O')
					{
						*ptr_data = (char) current_number;
					}
					else
					if (command == 'H')
					{
						*((uint16_t *) ptr_data) = (uint16_t) current_number;
					}
					else
*/					if (command == 'W')
					{
						*((int *) ptr_data) = current_number;
					}
/*					else if (command == 'o')
					{
						sam_ba_putdata_term(ptr_data, 1);
					}
					else if (command == 'h')
					{
						current_number = *((uint16_t *) ptr_data);
						sam_ba_putdata_term((uint8_t*) &current_number, 2);
					}
*/					else if (command == 'w')
					{
						current_number = *((uint32_t *) ptr_data);
						sam_ba_putdata_term((uint8_t*) &current_number, 4);
					}

					else if (command == 'G')
					{
						call_applet(current_number);
						__builtin_unreachable();
					}
/*					else if (command == 'T')
					{
						b_terminal_mode = 1;
						print_new_line();
					}
					else if (command == 'N')
					{
						//if (b_terminal_mode == 0)
						//{
						HND(putdata)((uint8_t *) RomBOOT_Version+2, 2);
						//	print_new_line();
						//}
						//b_terminal_mode = 0;
					}
*/					else if (command == 'V')
					{
						HND(putdata)((uint8_t *) RomBOOT_Version, sizeof(RomBOOT_Version));	//strlen(RomBOOT_Version));
					}

					command = 0;
					current_number = 0;

					if (b_terminal_mode)
					{
						HND(putdata)(">", 1);
					}
				}
				else
				{
/*
					if (('0' <= *ptr) && (*ptr <= '9'))
					{
						current_number = (current_number << 4) | (*ptr - '0');
					}
					else if (('A' <= *ptr) && (*ptr <= 'F'))
					{
						current_number = (current_number << 4) | (*ptr - 'A' + 0xa);
					}
					else if (('a' <= *ptr) && (*ptr <= 'f'))
					{
						current_number = (current_number << 4) | (*ptr - 'a' + 0xa);
					}
					else if (*ptr == ',')
					{
						ptr_data = (uint8_t *) current_number;
						current_number = 0;
					}
					else
					{
						command = *ptr;
						current_number = 0;
					}
*/
					if (*ptr == ',')
					{
						ptr_data = (uint8_t *) current_number;
						current_number = 0;
					}
					else
					{
						int val =  htoi(*ptr);
						if (val < 0) {
							command = *ptr;
							current_number = 0;
						}else{
							current_number = (current_number << 4) | val;
						}
					}
				}
				ptr++;
			}
		}
	}
}
