/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Enable the interfaces to save code size */
#define SAM_BA_BOTH_INTERFACES				0
#define SAM_BA_UART_ONLY					1
#define SAM_BA_USB_ONLY						2
#define DEFAULT_APP_START_ADDRESS			0x2000


#if defined(__SAMD21J18A__)
	#include "device_config_samd21j18a.h"
	#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAML21J18B__)
	#include "device_config_saml21j18b.h"
	#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAML22N18A__)
	#include "device_config_saml22n18a.h"
	#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAMC21J18A__)
	#include "device_config_samc21j18a.h"
	#define SAM_BA_INTERFACE	SAM_BA_UART_ONLY
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAMDA1J16A__)
	#include "device_config_samda1j16a.h"
	#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAMD51P20A__)
#include "device_config_same54p20a.h"
#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#elif defined (__SAME54P20A__)
#include "device_config_same54p20a.h"
#define SAM_BA_INTERFACE	SAM_BA_BOTH_INTERFACES
#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#else
	#error Unknown part number... Define part number and create device_config_xx.h
#endif

#define SAM_BA_VERSION						"v2.18"__DATE__" "__TIME__"\n\r"

#ifndef SAM_BA_INTERFACE
	#define SAM_BA_INTERFACE	SAM_BA_UART_ONLY
#endif

#ifndef APP_START_ADDRESS
	#define APP_START_ADDRESS	DEFAULT_APP_START_ADDRESS
#endif

#if SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
	#define SAM_BA_UART_INTERFACE_ENABLED		true
	#define SAM_BA_USB_INTERFACE_ENABLED		true
#elif SAM_BA_INTERFACE == SAM_BA_UART_ONLY
	#define SAM_BA_UART_INTERFACE_ENABLED		true
	#define SAM_BA_USB_INTERFACE_ENABLED		false
#elif SAM_BA_INTERFACE == SAM_BA_USB_ONLY
	#define SAM_BA_UART_INTERFACE_ENABLED		false
	#define SAM_BA_USB_INTERFACE_ENABLED		true
#else
	#error Atleast one SAM-BA interface should be enabled
#endif

#define cpu_irq_enable()				\
do {                                \
	__DMB();                        \
	__enable_irq();                 \
} while (0)

#define cpu_irq_disable()				\
do {                                \
	__disable_irq();                \
	__DMB();                        \
} while (0)

#define MIN(a, b)           (((a) < (b)) ?  (a) : (b))


#endif