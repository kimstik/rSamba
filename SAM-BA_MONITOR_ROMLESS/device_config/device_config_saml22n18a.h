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
	#include <iosaml22.h>
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

#define CPU_FREQUENCY						8000000
#define FLASH_WAIT_STATES					1

#define BOOT_LOAD_PIN						PIN_PC01
#define BOOT_PIN_MASK						(1U << (BOOT_LOAD_PIN & 0x1f))

#define PINMUX_UNUSED						0xFFFFFFFF
#define BOOT_USART_MODULE					SERCOM4
#define BOOT_USART_MUX_SETTINGS				(SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1))
#define BOOT_USART_PAD0						PINMUX_UNUSED
#define BOOT_USART_PAD1						PINMUX_UNUSED
#define BOOT_USART_PAD2						PINMUX_PC24D_SERCOM4_PAD2
#define BOOT_USART_PAD3						PINMUX_PC25D_SERCOM4_PAD3

/* Baud rate 115200 - clock 8MHz -> BAUD value-50436 */
#define USART_BAUD_REG_VAL_FOR_SAMBA		50436

#define NVM_USB_PAD_TRANSN_POS				13
#define NVM_USB_PAD_TRANSN_SIZE				5
#define NVM_USB_PAD_TRANSP_POS				18
#define NVM_USB_PAD_TRANSP_SIZE				5
#define NVM_USB_PAD_TRIM_POS				23
#define NVM_USB_PAD_TRIM_SIZE				3
#define NVM_SW_CALIB_DFLL48M_COARSE_POS		26
#define NVM_SW_CALIB_DFLL48M_COARSE_SIZE	6

#if DEBUG_ENABLE
	#define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
	#define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
	#define DEBUG_PIN_HIGH
	#define DEBUG_PIN_LOW
#endif



static inline void enable_sercom_digital_interface_clock(uint8_t inst)
{
	MCLK->APBCMASK.reg |= (1u << (inst + MCLK_APBCMASK_SERCOM0_Pos));
}
static inline void enable_usb_digital_interface_clock(void)
{
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
}
static inline void clock_configuration_for_usb(void)
{
	OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(4) | OSC32KCTRL_XOSC32K_RUNSTDBY | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_ENABLE;
	while(!(OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY));
	
	GCLK->GENCTRL[GCLK_PCHCTRL_GEN_GCLK1_Val].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC32K_Val) | GCLK_GENCTRL_GENEN;
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK1_Val));
	
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK1_Val) | GCLK_PCHCTRL_CHEN;
	while (!(GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg & GCLK_PCHCTRL_CHEN));

	uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP5) + (NVM_SW_CALIB_DFLL48M_COARSE_POS / 32))
		>> (NVM_SW_CALIB_DFLL48M_COARSE_POS % 32)) & ((1 << NVM_SW_CALIB_DFLL48M_COARSE_SIZE) - 1);
	/* In some revision chip, the Calibration value is not correct */
	if (coarse == 0x3f) {
		coarse = 0x1f;
	}

	OSCCTRL->DFLLCTRL.bit.ONDEMAND = false;
	while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

	OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_MUL(48000000/32768) | OSCCTRL_DFLLMUL_FSTEP(0xFF/8) | OSCCTRL_DFLLMUL_CSTEP(0x1F/8);
	OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_FINE(512) | OSCCTRL_DFLLVAL_COARSE(coarse);
	while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));
	OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_MODE | OSCCTRL_DFLLCTRL_ENABLE;
	while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY));

	GCLK->GENCTRL[GCLK_PCHCTRL_GEN_GCLK3_Val].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC16M_Val) | GCLK_GENCTRL_GENEN;
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK3_Val));

	GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK3_Val) | GCLK_PCHCTRL_CHEN;
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(GCLK_PCHCTRL_GEN_GCLK1_Val));
}

static inline void clock_configuration_for_boot_usart(void)
{
	uint8_t inst = 0;
	Sercom *sercom_instances[SERCOM_INST_NUM] = SERCOM_INSTS;

	for (uint32_t i = 0; i < SERCOM_INST_NUM; i++) {
		if (BOOT_USART_MODULE == sercom_instances[i]) {
			inst = i;
			break;
		}
	}

	enable_sercom_digital_interface_clock(inst);

	/* Set GCLK_GEN0 as source for GCLK_ID_SERCOMx_CORE */
	GCLK->PCHCTRL[inst+SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK0_Val);
	GCLK->PCHCTRL[inst+SERCOM0_GCLK_ID_CORE].reg |= GCLK_PCHCTRL_CHEN;
	/* Wait for clock synchronization */
	while (!(GCLK->PCHCTRL[inst+SERCOM0_GCLK_ID_CORE].reg & GCLK_PCHCTRL_CHEN));
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
	volatile uint32_t boot_en;

	/* Enable the input mode in Boot GPIO Pin */
	boot_port->DIRCLR.reg = BOOT_PIN_MASK;
	boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	boot_port->OUTSET.reg = BOOT_PIN_MASK;
	
	/* Read the BOOT_LOAD_PIN status... Reading twice to ensure bus has updated value */
	boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;
	boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;
	
	return(boot_en == 0);
}
static inline void set_flash_wait_states(void)
{
	/* Configure flash wait states */
	NVMCTRL->CTRLB.bit.RWS = FLASH_WAIT_STATES;
}

static inline void select_clock_source(void)
{
	/* Select Clock source... Enables 8MHz*/
	OSCCTRL->OSC16MCTRL.bit.FSEL = 1;

	/* Clear performance level status */
	PM->INTFLAG.reg = PM_INTFLAG_PLRDY;

	/* Switch performance level */
	PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2;

	/* Waiting performance level ready */
	while (!PM->INTFLAG.reg) {
		;
	}


	/* Assumed that this executes right after reset, hence other configurations as left with reset defaults*/
}
static inline void system_init(void)
{
	set_flash_wait_states();
	select_clock_source();
}
static inline void configure_usb_port_pins(void)
{
	configure_port_pin(PIN_PA24G_USB_DM, MUX_PA24G_USB_DM);
	configure_port_pin(PIN_PA25G_USB_DP, MUX_PA25G_USB_DP);
}
static inline void load_usb_pin_pad_calibration_values(void)
{
	uint32_t pad_transn, pad_transp, pad_trim;
	
	/* Load Pad Calibration */
	pad_transn =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSN_POS / 32))
		>> (NVM_USB_PAD_TRANSN_POS % 32)) & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

	if (pad_transn == 0x1F) {
		pad_transn = 5;
	}

	USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;

	pad_transp =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRANSP_POS / 32))
		>> (NVM_USB_PAD_TRANSP_POS % 32)) & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

	if (pad_transp == 0x1F) {
		pad_transp = 29;
	}

	USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;
	
	pad_trim =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_USB_PAD_TRIM_POS / 32))
		>> (NVM_USB_PAD_TRIM_POS % 32)) & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

	if (pad_trim == 0x7) {
		pad_trim = 3;
	}

	USB->DEVICE.PADCAL.bit.TRIM = pad_trim;
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