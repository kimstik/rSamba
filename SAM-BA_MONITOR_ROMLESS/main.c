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


/**
 * --------------------
 * SAM-BA Implementation 
 * --------------------
 *
 * Memory Mapping
 * --------------------
 * SAM-BA code will be located at 0x0 and executed before any applicative code.
 *
 * Applications compiled to be executed along with the bootloader will start at
 * 0x2000
 * Before jumping to the application, the bootloader changes the VTOR register
 * to use the interrupt vectors of the application @0x2000.<- not required as
 * application code is taking care of this
 * Version		2.18
 * Date			12-8-2017
 * Author		RB
*/

#include "sam_ba_monitor.h"
#include "device_config.h"

static void check_start_application(void);

/**
 *  \brief SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	DEBUG_PIN_HIGH;
	
	/* Jump in application if condition is satisfied */
	check_start_application();

	/* We have determined we should stay in the monitor. */
	/* System initialization */
	system_init();
	cpu_irq_enable();
	init_sam_ba_monitor_interface();
	
	DEBUG_PIN_LOW;
        
	/* Wait for a complete enum on usb or a '#' char on serial line */
	while (1) {
		process_sam_ba_monitor();
	}
}

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
	uint32_t app_start_address;

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);

	/**
	 * Test reset vector of application @APP_START_ADDRESS+4
	 * Stay in SAM-BA if *(APP_START+0x4) == 0xFFFFFFFF
	 * Application erased condition
	 */
//	if (app_start_address == 0xFFFFFFFF) {
	if (app_start_address>>18) {	//bit safer: value in range 256k
		/* Stay in Bootloader */
		return;
	}

	if(b_is_bootloader_condition_enabled())
	{
		/* Stay in Bootloader */
		return;
	}
	else
	{
		/* Re-base the Stack Pointer */
		__set_MSP(*(uint32_t *) APP_START_ADDRESS);

		/* Re-base the vector table base address */
		SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

		/* Jump to application Reset Handler in the application */
		asm("bx %0"::"r"(app_start_address));
	}
}
