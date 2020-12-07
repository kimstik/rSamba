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

#ifndef DEVICE_INCLUDE_H
#define DEVICE_INCLUDE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
	#include <iosame54.h>
	#define barrier()					__DMB()
	#define COMPILER_PRAGMA(arg)        _Pragma(#arg)
	#define COMPILER_ALIGNED(a)			COMPILER_PRAGMA(data_alignment = a)
	#define COMPILER_WORD_ALIGNED       COMPILER_PRAGMA(data_alignment = 4)
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
	#include "sam.h"
	#define COMPILER_ALIGNED(a)			__attribute__((__aligned__(a)))
	#define COMPILER_WORD_ALIGNED       __attribute__((__aligned__(4)))
#else
	#error Compiler Macro is unknown
#endif

#define CPU_FREQUENCY						48000000
#define FLASH_WAIT_STATES					1

#define BOOT_LOAD_PIN						PIN_PB31
#define BOOT_PIN_MASK						(1U << (BOOT_LOAD_PIN & 0x1f))

#define PINMUX_UNUSED						0xFFFFFFFF
#define BOOT_USART_MODULE					SERCOM2
#define BOOT_USART_MUX_SETTINGS				(SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(0))
#define BOOT_USART_PAD0						PINMUX_PB25D_SERCOM2_PAD0
#define BOOT_USART_PAD1						PINMUX_PB24D_SERCOM2_PAD1
#define BOOT_USART_PAD2						PINMUX_UNUSED
#define BOOT_USART_PAD3						PINMUX_UNUSED
#define UART_BAUDRATE						115200
#define UART_SERCOM_APBBMASK				MCLK_APBBMASK_SERCOM2
#define UART_SERCOM_GCLK_ID					SERCOM2_GCLK_ID_CORE
/* Baud rate 115200 - clock 8MHz -> BAUD value-50436 */
#define USART_BAUD_REG_VAL_FOR_SAMBA		63019

#define NVM_USB_PAD_TRANSN_POS				32
#define NVM_USB_PAD_TRANSN_SIZE				5
#define NVM_USB_PAD_TRANSP_POS				37
#define NVM_USB_PAD_TRANSP_SIZE				5
#define NVM_USB_PAD_TRIM_POS				42
#define NVM_USB_PAD_TRIM_SIZE				3
#define NVM_SW_CALIB_DFLL48M_COARSE_POS		
#define NVM_SW_CALIB_DFLL48M_COARSE_SIZE

#if DEBUG_ENABLE
	#define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
	#define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
	#define DEBUG_PIN_HIGH
	#define DEBUG_PIN_LOW
#endif



static inline void enable_sercom_digital_interface_clock(void)
{
	MCLK->APBBMASK.reg |= ((UART_SERCOM_APBBMASK));
}
static inline void enable_usb_digital_interface_clock(void)
{
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
	MCLK->AHBMASK.reg  |= MCLK_AHBMASK_USB;
}
static inline void clock_configuration_for_usb(void)
{
	GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK3_Val) | GCLK_PCHCTRL_CHEN;
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK3_Val));
}

static inline void clock_configuration_for_boot_usart(void)
{
	enable_sercom_digital_interface_clock();

	/* Set GCLK_GEN0 as source for GCLK_ID_SERCOMx_CORE */
	GCLK->PCHCTRL[UART_SERCOM_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK0_Val);
	GCLK->PCHCTRL[UART_SERCOM_GCLK_ID].reg |= GCLK_PCHCTRL_CHEN;
	/* Wait for clock synchronization */
	while (!(GCLK->PCHCTRL[UART_SERCOM_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));
}

static inline void configure_port_pin(uint32_t gpio_pin, uint32_t pin_mux_position)
{
	Port *const ports[PORT_INST_NUM] = PORT_INSTS;
	PortGroup *port;
	uint32_t pin_mask, pin_cfg;

	port = &(ports[gpio_pin/128]->Group[gpio_pin/32]);
	pin_cfg = PORT_WRCONFIG_PMUXEN | (pin_mux_position << PORT_WRCONFIG_PMUX_Pos) | PORT_WRCONFIG_INEN;
	pin_mask = 1L << ((gpio_pin) % 32);
	
	port->WRCONFIG.reg = ((pin_mask & 0xFFFF) << PORT_WRCONFIG_PINMASK_Pos) |
	pin_cfg | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG;
	port->WRCONFIG.reg = ((pin_mask >> 16) << PORT_WRCONFIG_PINMASK_Pos) |
	pin_cfg | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_HWSEL;
}

static inline void configure_sercom_port_pins(void)
{
	uint32_t gpio_pin, pin_mux_position;
	
	if(BOOT_USART_PAD0 != PINMUX_UNUSED){
		gpio_pin = BOOT_USART_PAD0 >> 16;
		pin_mux_position = BOOT_USART_PAD0 & 0xFFFF;
		configure_port_pin(gpio_pin, pin_mux_position);
	}

	if(BOOT_USART_PAD1 != PINMUX_UNUSED){
		gpio_pin = BOOT_USART_PAD1 >> 16;
		pin_mux_position = BOOT_USART_PAD1 & 0xFFFF;
		configure_port_pin(gpio_pin, pin_mux_position);
	}

	if(BOOT_USART_PAD2 != PINMUX_UNUSED){
		gpio_pin = BOOT_USART_PAD2 >> 16;
		pin_mux_position = BOOT_USART_PAD2 & 0xFFFF;
		configure_port_pin(gpio_pin, pin_mux_position);
	}

	if(BOOT_USART_PAD3 != PINMUX_UNUSED){
		gpio_pin = BOOT_USART_PAD3 >> 16;
		pin_mux_position = BOOT_USART_PAD3 & 0xFFFF;
		configure_port_pin(gpio_pin, pin_mux_position);
	}
}

static inline bool b_is_bootloader_condition_enabled(void)
{
	volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
	bool boot_en;

	/* Enable the input mode in Boot GPIO Pin */
	boot_port->DIRCLR.reg = BOOT_PIN_MASK;
	boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	boot_port->OUTSET.reg = BOOT_PIN_MASK;
	
	/* Read the BOOT_LOAD_PIN status */
	boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;
	
	return(boot_en == 0);
}
static inline void set_flash_wait_states(void)
{
	/* Configure flash wait states */
	//NVMCTRL->CTRLA.bit.AUTOWS = 1;
}

static inline void select_clock_source(void)
{
		OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(0) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN1K | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_ENABLE;
		while(!(OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY));
		GCLK->GENCTRL[GCLK_PCHCTRL_GEN_GCLK1_Val].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC32K_Val) | GCLK_GENCTRL_GENEN;
		while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK1_Val));
		
		GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK1_Val) | GCLK_PCHCTRL_CHEN;
		while (!(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg & GCLK_PCHCTRL_CHEN));

		OSCCTRL->DFLLCTRLA.bit.ONDEMAND = false;
		while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

		OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL(48000) | OSCCTRL_DFLLMUL_FSTEP(1) | OSCCTRL_DFLLMUL_CSTEP(1);
		while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));
		OSCCTRL->DFLLCTRLB.reg |= OSCCTRL_DFLLCTRLB_MODE | OSCCTRL_DFLLCTRLB_CCDIS | OSCCTRL_DFLLCTRLB_USBCRM;
		OSCCTRL->DFLLCTRLA.reg |= OSCCTRL_DFLLCTRLA_ENABLE;
		while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));
		PORT->Group[1].PINCFG[17].reg = 1;
		// Enable peripheral function 12 for PB14, refer chapter I/O Multiplexing in the device datasheet
		PORT->Group[1].PMUX[8].bit.PMUXO = 12;
		PORT->Group[1].PMUX[8].bit.PMUXE = 12;
		GCLK->GENCTRL[GCLK_PCHCTRL_GEN_GCLK3_Val].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | GCLK_GENCTRL_GENEN |GCLK_GENCTRL_OE;
		while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK3_Val));
		GCLK->GENCTRL[GCLK_PCHCTRL_GEN_GCLK0_Val].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | GCLK_GENCTRL_GENEN |GCLK_GENCTRL_OE;
		while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK0_Val));
}
static inline void system_init(void)
{
	set_flash_wait_states();
	select_clock_source();
}
static inline void configure_usb_port_pins(void)
{
	configure_port_pin(PIN_PA24H_USB_DM, MUX_PA24H_USB_DM);
	configure_port_pin(PIN_PA25H_USB_DP, MUX_PA25H_USB_DP);
}
static inline void load_usb_pin_pad_calibration_values(void)
{
	uint32_t pad_transn =( *((uint32_t *)(NVMCTRL_SW0)
	+ (NVM_USB_PAD_TRANSN_POS / 32))
	>> (NVM_USB_PAD_TRANSN_POS % 32))
	& ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);
	uint32_t pad_transp =( *((uint32_t *)(NVMCTRL_SW0)
	+ (NVM_USB_PAD_TRANSP_POS / 32))
	>> (NVM_USB_PAD_TRANSP_POS % 32))
	& ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);
	uint32_t pad_trim =( *((uint32_t *)(NVMCTRL_SW0)
	+ (NVM_USB_PAD_TRIM_POS / 32))
	>> (NVM_USB_PAD_TRIM_POS % 32))
	& ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);
	if (pad_transn == 0 || pad_transn == 0x1F) {
		pad_transn = 9;
	}
	if (pad_transp == 0 || pad_transp == 0x1F) {
		pad_transp = 25;
	}
	if (pad_trim == 0 || pad_trim == 0x7) {
		pad_trim = 6;
	}
	USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN(pad_transn) |
	USB_PADCAL_TRANSP(pad_transp) | USB_PADCAL_TRIM(pad_trim);	
}

static inline void wait_for_usart_enable_sync(Sercom* sercom)
{
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
}

static inline void wait_for_usart_swrst_sync(Sercom* sercom)
{
	while(sercom->USART.SYNCBUSY.bit.SWRST);
}

static inline void wait_for_usart_swrst_enable_sync(Sercom* sercom)
{
	while(sercom->USART.SYNCBUSY.bit.SWRST || sercom->USART.SYNCBUSY.bit.ENABLE);
}
static inline void wait_for_usart_ctrlb_sync(Sercom* sercom)
{
	while(sercom->USART.SYNCBUSY.bit.CTRLB);
}
static inline void wait_for_uart_syncbusy_clear(Sercom* sercom)
{
	while(sercom->USART.SYNCBUSY.reg);
}
static inline void uart_read_clear_errors(Sercom* sercom)
{
	/* Check for errors */
	if((uint8_t)(sercom->USART.STATUS.reg & SERCOM_USART_STATUS_MASK))
	{
		sercom->USART.STATUS.reg &= 0xFFF8;
	}
}
#endif