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


#include <stdint.h>
#include "cdc_enumerate.h"
#include "device_config.h"

#if SAM_BA_USB_INTERFACE_ENABLED

COMPILER_WORD_ALIGNED UsbDeviceDescriptor usb_endpoint_table[MAX_EP];
COMPILER_WORD_ALIGNED uint8_t udd_ep_out_cache_buffer[2][64]; //1 for CTRL, 1 for BULK
COMPILER_WORD_ALIGNED uint8_t udd_ep_in_cache_buffer[2][64]; //1 for CTRL, 1 for BULK

COMPILER_WORD_ALIGNED
const char devDescriptor[] = {
	/* Device descriptor */
	0x12,   // bLength
	0x01,   // bDescriptorType
	0x10,   // bcdUSBL
	0x01,   //
	0x02,   // bDeviceClass:    CDC class code
	0x00,   // bDeviceSubclass: CDC class sub code
	0x00,   // bDeviceProtocol: CDC Device protocol
	0x40,   // bMaxPacketSize0
	0xEB,   // idVendorL
	0x03,   //
	0x24,   // idProductL
	0x61,   //
	0x10,   // bcdDeviceL
	0x01,   //
	0x00,   // iManufacturer    // 0x01
	0x00,   // iProduct
	0x00,   // SerialNumber
	0x01    // bNumConfigs
};

COMPILER_WORD_ALIGNED
char cfgDescriptor[] = {
	/* ============== CONFIGURATION 1 =========== */
	/* Configuration 1 descriptor */
	0x09,   // CbLength
	0x02,   // CbDescriptorType
	0x43,   // CwTotalLength 2 EP + Control
	0x00,
	0x02,   // CbNumInterfaces
	0x01,   // CbConfigurationValue
	0x00,   // CiConfiguration
	0xC0,   // CbmAttributes 0xA0
	0x00,   // CMaxPower

	/* Communication Class Interface Descriptor Requirement */
	0x09, // bLength
	0x04, // bDescriptorType
	0x00, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x01, // bNumEndpoints
	0x02, // bInterfaceClass
	0x02, // bInterfaceSubclass
	0x00, // bInterfaceProtocol
	0x00, // iInterface

	/* Header Functional Descriptor */
	0x05, // bFunction Length
	0x24, // bDescriptor type: CS_INTERFACE
	0x00, // bDescriptor subtype: Header Func Desc
	0x10, // bcdCDC:1.1
	0x01,

	/* ACM Functional Descriptor */
	0x04, // bFunctionLength
	0x24, // bDescriptor Type: CS_INTERFACE
	0x02, // bDescriptor Subtype: ACM Func Desc
	0x00, // bmCapabilities

	/* Union Functional Descriptor */
	0x05, // bFunctionLength
	0x24, // bDescriptorType: CS_INTERFACE
	0x06, // bDescriptor Subtype: Union Func Desc
	0x00, // bMasterInterface: Communication Class Interface
	0x01, // bSlaveInterface0: Data Class Interface

	/* Call Management Functional Descriptor */
	0x05, // bFunctionLength
	0x24, // bDescriptor Type: CS_INTERFACE
	0x01, // bDescriptor Subtype: Call Management Func Desc
	0x00, // bmCapabilities: D1 + D0
	0x01, // bDataInterface: Data Class Interface 1

	/* Endpoint 1 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x83,   // bEndpointAddress, Endpoint 03 - IN
	0x03,   // bmAttributes      INT
	0x08,   // wMaxPacketSize
	0x00,
	0xFF,   // bInterval

	/* Data Class Interface Descriptor Requirement */
	0x09, // bLength
	0x04, // bDescriptorType
	0x01, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x02, // bNumEndpoints
	0x0A, // bInterfaceClass
	0x00, // bInterfaceSubclass
	0x00, // bInterfaceProtocol
	0x00, // iInterface

	/* First alternate setting */
	/* Endpoint 1 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x81,   // bEndpointAddress, Endpoint 01 - IN
	0x02,   // bmAttributes      BULK
	USB_EP_IN_SIZE,   // wMaxPacketSize
	0x00,
	0x00,   // bInterval

	/* Endpoint 2 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x02,   // bEndpointAddress, Endpoint 02 - OUT
	0x02,   // bmAttributes      BULK
	USB_EP_OUT_SIZE,   // wMaxPacketSize
	0x00,
	0x00    // bInterval
};


static usb_cdc_line_coding_t line_coding = {
  115200, // baudrate
  0,      // 1 Stop Bit
  0,      // None Parity
  8     // 8 Data bits  
};

USB_CDC pCdc;

/* USB standard request code */
#define STD_GET_STATUS_ZERO           0x0080
#define STD_GET_STATUS_INTERFACE      0x0081
#define STD_GET_STATUS_ENDPOINT       0x0082

#define STD_CLEAR_FEATURE_ZERO        0x0100
#define STD_CLEAR_FEATURE_INTERFACE   0x0101
#define STD_CLEAR_FEATURE_ENDPOINT    0x0102

#define STD_SET_FEATURE_ZERO          0x0300
#define STD_SET_FEATURE_INTERFACE     0x0301
#define STD_SET_FEATURE_ENDPOINT      0x0302

#define STD_SET_ADDRESS               0x0500
#define STD_GET_DESCRIPTOR            0x0680
#define STD_SET_DESCRIPTOR            0x0700
#define STD_GET_CONFIGURATION         0x0880
#define STD_SET_CONFIGURATION         0x0900
#define STD_GET_INTERFACE             0x0A81
#define STD_SET_INTERFACE             0x0B01
#define STD_SYNCH_FRAME               0x0C82

/* CDC Class Specific Request Code */
#define GET_LINE_CODING               0x21A1
#define SET_LINE_CODING               0x2021
#define SET_CONTROL_LINE_STATE        0x2221


static void AT91F_CDC_Enumerate(P_USB_CDC pCdc);


/**
 * \fn AT91F_InitUSB
 *
 * \brief Initializes USB
 */
void AT91F_InitUSB(void)
{

	enable_usb_digital_interface_clock();
	configure_usb_port_pins();
	
	/* Reset */
	USB->DEVICE.CTRLA.bit.SWRST = 1;
	while (USB->DEVICE.SYNCBUSY.bit.SWRST) {
		/* Sync wait */
	}

	load_usb_pin_pad_calibration_values();
	
	/* Set the configuration */
	/* Set mode to Device mode */
	USB->DEVICE.CTRLA.bit.MODE = 0;
	/* Enable Run in Standby */
	USB->DEVICE.CTRLA.bit.RUNSTDBY = true;
	/* Set the descriptor address */
	USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_endpoint_table[0]);
	/* Set speed configuration to Full speed */
	USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
	/* Attach to the USB host */
	USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH;

	/* Initialize endpoint table RAM location to a known value 0 */
	//memset((uint8_t *)(&usb_endpoint_table[0]), 0, sizeof(usb_endpoint_table));
	uint8_t* pdata = (uint8_t *)(&usb_endpoint_table[0]);
	for(uint32_t u32_index=0; u32_index < sizeof(usb_endpoint_table); u32_index++)
	{
		*pdata++ = 0;
	}
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_CDC_Open
//* \brief
//*----------------------------------------------------------------------------
P_USB_CDC AT91F_CDC_Open(P_USB_CDC pCdc,Usb *pUsb)
{
	pCdc->pUsb = pUsb;
	pCdc->currentConfiguration = 0;
	pCdc->currentConnection    = 0;
	pCdc->IsConfigured = USB_IsConfigured;
	pCdc->Write        = USB_Write;
	pCdc->Read         = USB_Read;
	pCdc->pUsb->DEVICE.CTRLA.bit.ENABLE = true;
	return pCdc;
}

//*----------------------------------------------------------------------------
//* \fn    USB_IsConfigured
//* \brief Test if the device is configured and handle enumerationDEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE1
//*----------------------------------------------------------------------------
uint8_t USB_IsConfigured(P_USB_CDC pCdc)
{
	Usb *pUsb = pCdc->pUsb;

	/* Check for End of Reset flag */
	if (pUsb->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST) {
		/* Clear the flag */
		pUsb->DEVICE.INTFLAG.reg |= USB_DEVICE_INTFLAG_EORST;
		/* Set Device address as 0 */
		pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;
		/* Configure endpoint 0 */
		/* Configure Endpoint 0 for Control IN and Control OUT */
		pUsb->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
		pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
		pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
		/* Configure control OUT Packet size to 64 bytes */
		usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
		/* Configure control IN Packet size to 64 bytes */
		usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
		/* Configure the data buffer address for control OUT */
		usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[0];
		/* Configure the data buffer address for control IN */
		usb_endpoint_table[0].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[0];
		/* Set Multipacket size to 8 for control OUT and byte count to 0*/
		usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
		usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
		pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

		// Reset current configuration value to 0
		pCdc->currentConfiguration = 0;
	} else if (pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP) {
		AT91F_CDC_Enumerate(pCdc);
	}

	return pCdc->currentConfiguration;
}


static volatile bool read_job = false;
//*----------------------------------------------------------------------------
//* \fn    USB_Read
//* \brief Read available data from Endpoint OUT
//*----------------------------------------------------------------------------
uint32_t USB_Read(P_USB_CDC pCdc, char *pData, uint32_t length)
{
	Usb *pUsb = pCdc->pUsb;
	uint32_t packetSize = 0;
	uint8_t* p_cache_buffer;

	if (!read_job) {
		/* Set the buffer address for ep data */
		usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[USB_EP_OUT-1];
		/* Set the byte count as zero */
		usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
		/* Set the byte count as zero */
		usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
		/* Start the reception by clearing the bank 0 ready bit */
		pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
		/* set the user flag */
		read_job = true;
	}

	/* Check for Transfer Complete 0 flag */
	if ( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0 ) {
		/* Set packet size */
		packetSize = MIN(usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT, length);
		/* Copy read data to user buffer */
		//memcpy(pData, udd_ep_out_cache_buffer[USB_EP_OUT-1], packetSize);
		p_cache_buffer = udd_ep_out_cache_buffer[USB_EP_OUT-1];
		for(uint32_t u32_index=0; u32_index < packetSize; u32_index++)
		{
			pData[u32_index] = p_cache_buffer[u32_index];
		}
		
		/* Clear the Transfer Complete 0 flag */
		pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg |= USB_DEVICE_EPINTFLAG_TRCPT0;
		/* Clear the user flag */
		read_job = false;
	}

	return packetSize;
}

uint32_t USB_Read_blocking(P_USB_CDC pCdc, char *pData, uint32_t length)
{
	Usb *pUsb = pCdc->pUsb;

	if (read_job) {
		/* Stop the reception by setting the bank 0 ready bit */
		pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.bit.BK0RDY = true;
		/* Clear the user flag */
		read_job = false;
	}

	/* Set the buffer address for ep data */
	usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = ((uint32_t)pData);
	/* Set the byte count as zero */
	usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	/* Set the multi packet size as zero for multi-packet transfers where length > ep size */
	usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = length;
	/* Clear the bank 0 ready flag */
	pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
	/* Wait for transfer to complete */
	while (!( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT0 ));
	/* Clear Transfer complete 0 flag */
	pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg |= USB_DEVICE_EPINTFLAG_TRCPT0;

	return length;

}


uint32_t USB_Write(P_USB_CDC pCdc, const char *pData, uint32_t length, uint8_t ep_num)
{
	Usb *pUsb = pCdc->pUsb;
	uint32_t data_address;
	uint8_t buf_index;
	uint8_t* p_cache_buffer;

	/* Set buffer index */
	buf_index = (ep_num == 0) ? 0 : 1;

	/* Check for requirement for multi-packet or auto zlp */
	if (length >= (1 << (usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.SIZE + 3))) {
		/* Update the EP data address */
		data_address = (uint32_t) pData;
		/* Enable auto zlp */
		usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = true;
	} else {
		/* Copy to local buffer */
		//memcpy(udd_ep_in_cache_buffer[buf_index], pData, length);
		p_cache_buffer = udd_ep_in_cache_buffer[buf_index];
		for(uint32_t u32_index=0; u32_index < length; u32_index++)
		{
			p_cache_buffer[u32_index] = pData[u32_index];
		}
		/* Update the EP data address */
		data_address = (uint32_t) &udd_ep_in_cache_buffer[buf_index];
	}

	/* Set the buffer address for ep data */
	usb_endpoint_table[ep_num].DeviceDescBank[1].ADDR.reg = data_address;
	/* Set the byte count as zero */
	usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = length;
	/* Set the multi packet size as zero for multi-packet transfers where length > ep size */
	usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	/* Clear the transfer complete flag  */
	pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg |= USB_DEVICE_EPINTFLAG_TRCPT1;
	/* Set the bank as ready */
	pUsb->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.bit.BK1RDY = true;
	
	/* Wait for transfer to complete */
	while (!( pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1 ));
	
	return length;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendData
//* \brief Send Data through the control endpoint
//*----------------------------------------------------------------------------

static void AT91F_USB_SendData(P_USB_CDC pCdc, const char *pData, uint32_t length)
{
	USB_Write(pCdc, pData, length, 0);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendZlp
//* \brief Send zero length packet through the control endpoint
//*----------------------------------------------------------------------------
void AT91F_USB_SendZlp(Usb *pUsb)
{
	/* Set the byte count as zero */
	usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
	/* Clear the transfer complete flag  */
	pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg |= USB_DEVICE_EPINTFLAG_TRCPT1;
	/* Set the bank as ready */
	pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = true;
	/* Wait for transfer to complete */
	while (!( pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1 ));
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendStall
//* \brief Stall the control endpoint
//*----------------------------------------------------------------------------
void AT91F_USB_SendStall(Usb *pUsb, bool direction_in)
{
	/* Check the direction */
	if (direction_in) {
		/* Set STALL request on IN direction */
		pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
	} else {
		/* Set STALL request on OUT direction */
		pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
	}
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_CDC_Enumerate
//* \brief This function is a callback invoked when a SETUP packet is received
//*----------------------------------------------------------------------------
void AT91F_CDC_Enumerate(P_USB_CDC pCdc)
{
	Usb *pUsb = pCdc->pUsb;
	uint8_t bmRequestType, bRequest, dir;
	uint16_t wValue, wIndex, wLength, wStatus;

	/* Clear the Received Setup flag */
	pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg |= USB_DEVICE_EPINTFLAG_RXSTP;

	/* Read the USB request parameters */
	bmRequestType = udd_ep_out_cache_buffer[0][0];
	bRequest      = udd_ep_out_cache_buffer[0][1];
	wValue        = (udd_ep_out_cache_buffer[0][2] & 0xFF);
	wValue       |= (udd_ep_out_cache_buffer[0][3] << 8);
	wIndex        = (udd_ep_out_cache_buffer[0][4] & 0xFF);
	wIndex       |= (udd_ep_out_cache_buffer[0][5] << 8);
	wLength       = (udd_ep_out_cache_buffer[0][6] & 0xFF);
	wLength      |= (udd_ep_out_cache_buffer[0][7] << 8);

	/* Clear the Bank 0 ready flag on Control OUT */
	pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

	/* Handle supported standard device request Cf Table 9-3 in USB specification Rev 1.1 */
	switch ((bRequest << 8) | bmRequestType) {
	case STD_GET_DESCRIPTOR:
		if (wValue == 0x100)
			/* Return Device Descriptor */
			AT91F_USB_SendData(pCdc, devDescriptor, MIN(sizeof(devDescriptor), wLength));
		else if (wValue == 0x200)
			/* Return Configuration Descriptor */
			AT91F_USB_SendData(pCdc, cfgDescriptor, MIN(sizeof(cfgDescriptor), wLength));
		else
			/* Stall the request */
			AT91F_USB_SendStall(pUsb, true);
		break;
	case STD_SET_ADDRESS:
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		/* Set device address to the newly received address from host */
		pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue;
		break;
	case STD_SET_CONFIGURATION:
		/* Store configuration */
		pCdc->currentConfiguration = (uint8_t)wValue;
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		/* Configure BULK OUT endpoint for CDC Data interface*/
		pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(3);
		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
		pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
		/* Configure the data buffer */
		usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[1];
		/* Configure BULK IN endpoint for CDC Data interface */
		pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(3);
		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
		pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
		/* Configure the data buffer */
		usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[1];
		/* Configure INTERRUPT IN endpoint for CDC COMM interface*/
		pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(4);
		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[USB_EP_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0;
		pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
		break;
	case STD_GET_CONFIGURATION:
		/* Return current configuration value */
		AT91F_USB_SendData(pCdc, (char *) &(pCdc->currentConfiguration), sizeof(pCdc->currentConfiguration));
		break;
	case STD_GET_STATUS_ZERO:
		wStatus = 0;
		AT91F_USB_SendData(pCdc, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_INTERFACE:
		wStatus = 0;
		AT91F_USB_SendData(pCdc, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_ENDPOINT:
		wStatus = 0;
		dir = wIndex & 80;
		wIndex &= 0x0F;
		if (wIndex <= 3) {
			if (dir) {
				wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) ? 1 : 0;
			} else {
				wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) ? 1 : 0;
			}
			/* Return current status of endpoint */
			AT91F_USB_SendData(pCdc, (char *) &wStatus, sizeof(wStatus));
		}
		else
			/* Stall the request */
			AT91F_USB_SendStall(pUsb, true);
		break;
	case STD_SET_FEATURE_ZERO:
		/* Stall the request */
		AT91F_USB_SendStall(pUsb, true);
		break;
	case STD_SET_FEATURE_INTERFACE:
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		break;
	case STD_SET_FEATURE_ENDPOINT:
		dir = wIndex & 0x80;
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			/* Set STALL request for the endpoint */
			if (dir) {
				pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
			} else {
				pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
			}
			/* Send ZLP */
			AT91F_USB_SendZlp(pUsb);
		}
		else
			/* Stall the request */
			AT91F_USB_SendStall(pUsb, true);
		break;
	case STD_CLEAR_FEATURE_ZERO:
		/* Stall the request */
		AT91F_USB_SendStall(pUsb, true);
		break;
	case STD_CLEAR_FEATURE_INTERFACE:
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		break;
	case STD_CLEAR_FEATURE_ENDPOINT:
		dir = wIndex & 0x80;
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			if (dir) {
				if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) {
					// Remove stall request
					pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
					if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
						pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
						// The Stall has occurred, then reset data toggle
						pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
					}
				}
			} else {
				if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) {
					// Remove stall request
					pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
					if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
						pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
						// The Stall has occurred, then reset data toggle
						pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
					}
				}
			}
			/* Send ZLP */
			AT91F_USB_SendZlp(pUsb);
		}
		else {
			AT91F_USB_SendStall(pUsb, true);
		}
		break;

	// handle CDC class requests
	case SET_LINE_CODING:
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		break;
	case GET_LINE_CODING:
		/* Send current line coding */
		AT91F_USB_SendData(pCdc, (char *) &line_coding, MIN(sizeof(usb_cdc_line_coding_t), wLength));
		break;
	case SET_CONTROL_LINE_STATE:
		/* Store the current connection */
		pCdc->currentConnection = wValue;
		/* Send ZLP */
		AT91F_USB_SendZlp(pUsb);
		break;
	default:
		/* Stall the request */
		AT91F_USB_SendStall(pUsb, true);
		break;
	}
}
#endif