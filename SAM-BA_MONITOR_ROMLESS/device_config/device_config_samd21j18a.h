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
	#include <iosamd21.h>
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

#define BOOT_LOAD_PIN						PIN_PA15
#define BOOT_PIN_MASK						(1U << (BOOT_LOAD_PIN & 0x1f))

#define PINMUX_UNUSED						0xFFFFFFFF
#define BOOT_USART_MODULE					SERCOM3
#define BOOT_USART_MUX_SETTINGS				(SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(0))
#define BOOT_USART_PAD0						PINMUX_PA22C_SERCOM3_PAD0
#define BOOT_USART_PAD1						PINMUX_PA23C_SERCOM3_PAD1
#define BOOT_USART_PAD2						PINMUX_UNUSED
#define BOOT_USART_PAD3						PINMUX_UNUSED

/* Baud rate 115200 - clock 8MHz -> BAUD value-50436 */
#define USART_BAUD_REG_VAL_FOR_SAMBA		50436

#define NVM_USB_PAD_TRANSN_POS				45
#define NVM_USB_PAD_TRANSN_SIZE				5
#define NVM_USB_PAD_TRANSP_POS				50
#define NVM_USB_PAD_TRANSP_SIZE				5
#define NVM_USB_PAD_TRIM_POS				55
#define NVM_USB_PAD_TRIM_SIZE				3
#define NVM_SW_CALIB_DFLL48M_COARSE_VAL		58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL		64


#if DEBUG_ENABLE
	#define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
	#define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
	#define DEBUG_PIN_HIGH
	#define DEBUG_PIN_LOW
#endif



static inline void enable_sercom_digital_interface_clock(uint8_t inst)
{
	PM->APBCMASK.reg |= (1u << (inst + PM_APBCMASK_SERCOM0_Pos));
}
static inline void enable_usb_digital_interface_clock(void)
{
	PM->APBBMASK.reg |= PM_APBBMASK_USB;
}
static inline void clock_configuration_for_usb(void)
{
	uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32))
		>> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32)) & ((1 << 6) - 1);
	if (coarse == 0x3f) {
		coarse = 0x1f;
	}

	uint32_t fine =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_FINE_VAL / 32))
		>> (NVM_SW_CALIB_DFLL48M_FINE_VAL % 32)) & ((1 << 10) - 1);
	if (fine == 0x3ff) {
		fine = 0x1ff;
	}

	SYSCTRL->DFLLCTRL.bit.ONDEMAND = false;
	while(!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));
	
	SYSCTRL->DFLLMUL.reg = 48000;
	SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_FINE(fine) | SYSCTRL_DFLLVAL_COARSE(coarse);
	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_ENABLE;

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_DFLL48_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK0_Val) | GCLK_CLKCTRL_CLKEN;
	
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(GCLK_CLKCTRL_GEN_GCLK1_Val);
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val) | GCLK_GENCTRL_GENEN;
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_USB_Val) | GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK1_Val) | GCLK_CLKCTRL_CLKEN;
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
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(inst+GCLK_CLKCTRL_ID_SERCOM0_CORE_Val) | (GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK0_Val)) | GCLK_CLKCTRL_CLKEN;
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
	 SYSCTRL->OSC8M.bit.PRESC = 0;

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