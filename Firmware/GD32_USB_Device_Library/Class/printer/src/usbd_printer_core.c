/**
  ******************************************************************************
  * @file    usbd_printer_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB Printing device class core functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------ */
#include "usbd_printer_core.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_PRINTER_Class
  * @{
  */
static uint8_t  USBD_PRINTER_Init               (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_PRINTER_DeInit             (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_PRINTER_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_PRINTER_DataOut            (void *pudev, uint8_t EpID);
static uint8_t* USBD_PRINTER_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);

/** @defgroup USB_PRINTER_Class_Private_Varibales
  * @{
  */
static uint32_t USBD_PRINTER_AltSet = 0;

/* Printer port status: paper not empty/selected/no error */
static uint8_t PortStatus = 0x18;

uint8_t PrinterDataBuf[PRINTER_OUT_PACKET];

/**
  * @}
  */

/** @defgroup USB_HID_Class_Exported_Variables
  * @{
  */
uint8_t DEVICE_ID[DEVICE_ID_LEN] =
{
    0x00, 0x66,
    'M', 'A', 'N', 'U', 'F', 'A', 'C', 'T', 'U', 'R', 'E', 'R', ':',
    'G', 'I', 'G', 'A', ' ', 'D', 'E', 'V', 'I', 'C', 'E', ';',
    'C', 'O', 'M', 'M', 'A', 'N', 'D', ' ', 'S', 'E', 'T', ':',
    'P', 'C', 'L', ',', 'M', 'P', 'L', ';',
    'M', 'O', 'D', 'E', 'L', ':',
    'L', 'a', 's', 'e', 'r', 'B', 'e', 'a', 'm', '?', ';',
    'C', 'O', 'M', 'M', 'E', 'N', 'T', ':',
    'G', 'o', 'o', 'd', ' ', '!', ';',
    'A', 'C', 'T', 'I', 'V', 'E', ' ', 'C', 'O', 'M', 'M', 'A', 'N', 'D', ' ', 'S', 'E', 'T', ':',
    'P', 'C', 'L', ';'
};

USBD_Class_cb_TypeDef  USBD_Printer_cb = 
{
    USBD_PRINTER_Init,
    USBD_PRINTER_DeInit,
    USBD_PRINTER_GetClassDescriptor,
    USBD_PRINTER_ClassReqHandle,
    USBD_PRINTER_GetInterface,
    USBD_PRINTER_SetInterface,
    NULL, /* EP0_TxSent */
    NULL, /* EP0_RxReady */
    USBD_PRINTER_DataIn,
    USBD_PRINTER_DataOut,
    NULL, /* SOF */
    USBD_PRINTER_GetCfgDesc,
};

/* USB Printing device configuration descriptor set */
const uint8_t USBD_PRINTER_CfgDesc[USB_PRINTER_CONFIG_DESC_SIZE] =
{
    0x09,         /* bLength: configuration descriptor size */
    USB_DESCTYPE_CONFIGURATION, /* bDescriptorType: configuration descriptor type */
    USB_PRINTER_CONFIG_DESC_SIZE, /* wTotalLength: configuration descriptor set total length */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: configuration value */
    0x00,         /* iConfiguration: index of string descriptor describing the configuration */
    0xA0,         /* bmAttributes: device attributes (bus powered and support remote wakeup) */
    0x32,         /* bMaxPower 100 mA: this current is used for detecting Vbus */

    /************** interface descriptor  ****************/
    0x09,         /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE,/* bDescriptorType: interface descriptor type */
    0x00,         /* bInterfaceNumber: number of interface */
    0x00,         /* bAlternateSetting: alternate setting */
    0x02,         /* bNumEndpoints: use 2 endpoints for Tx/Rx */
    0x07,         /* bInterfaceClass: Printing class */
    0x01,         /* bInterfaceSubClass: 01 = Printers */
    0x02,         /* nInterfaceProtocol: 02 = Bi-directional interface */
    0x00,         /* iInterface: index of interface string descriptor */

    /******************** Printer IN endpoint descriptor ********************/
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    PRINTER_IN_EP,         /* bEndpointAddress: endpoint address (EP1_IN) */
    0x02,                  /* bmAttributes: endpoint attribute(bulk endpoint) */
    PRINTER_IN_PACKET,     /* wMaxPacketSize: 64 bytes max */
    0x00,
    0x00,                  /* bInterval: polling interval */

    /******************** Printer OUT endpoint descriptor ********************/
    0x07,          /* bLength:   Descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    PRINTER_OUT_EP,        /* bEndpointAddress: endpoint address (EP1_OUT) */
    0x02,                  /* bmAttributes: endpoint attribute(bulk endpoint) */
    PRINTER_OUT_PACKET,    /* wMaxPacketSize: 64 bytes max */
    0x00,
    0x00,                  /* bInterval: polling interval */
};

/**
  * @}
  */

/** @defgroup USB_HID_Class_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the PRINTER device
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_Init (void *pudev, uint8_t ConfigIndex)
{
    USB_EP_BufConfig(pudev, PRINTER_IN_EP, USB_SNG_BUFTYPE, PRINTER_TX_ADDRESS);
    USB_EP_BufConfig(pudev, PRINTER_OUT_EP, USB_SNG_BUFTYPE, PRINTER_RX_ADDRESS);

    /* Initialize Tx endpoint */
    USB_EP_Init(pudev,
                PRINTER_IN_EP,
                USB_EPTYPE_BULK,
                PRINTER_IN_PACKET);

    /* Initialize Rx endpoint */
    USB_EP_Init(pudev,
                PRINTER_OUT_EP,
                USB_EPTYPE_BULK,
                PRINTER_OUT_PACKET);

    return USBD_OK;
}

/**
  * @brief  De-Initialize the PRINTER device
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: Configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_DeInit (void *pudev, uint8_t ConfigIndex)
{
    /* Deinitialize HID endpoints */
    USB_EP_DeInit (pudev, PRINTER_IN_EP);
    USB_EP_DeInit (pudev, PRINTER_OUT_EP);

    return USBD_OK;
}

/**
  * @brief  Handle the PRINTER class-specific requests
  * @param  pudev: pointer to usb device instance
  * @param  req: device class-specific request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    switch (req->bRequest)
    {
        case GET_DEVICE_ID:
            USB_CtlTx(pudev, DEVICE_ID, DEVICE_ID_LEN);
            break;

        case GET_PORT_STATUS:
            USB_CtlTx (pudev, (uint8_t *)&PortStatus, 1);
            break;

        case SOFT_RESET:
            USB_EP_Rx(pudev, PRINTER_OUT_EP, PrinterDataBuf, PRINTER_OUT_PACKET);
            break;

        default:
            USBD_EnumError (pudev, req);
            return USBD_FAIL; 
    }

    return USBD_OK;
}

/**
  * @brief  Get PRINTER class descriptor(Report descriptor)
  * @param  pudev: pointer to usb device instance
  * @param  req: device request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USB_CtlTx (pudev, (uint8_t *)&USBD_PRINTER_AltSet, 1);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USBD_PRINTER_AltSet = (uint8_t)(req->wValue);

    return USBD_OK;
}

/**
  * @brief  Data sent on non-control IN endpoint
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_DataIn (void *pudev, uint8_t EpID)
{
    /* Added by user */

    return USBD_OK;
}

/**
  * @brief  Data receive on non-control IN endpoint
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_DataOut (void *pudev, uint8_t EpID)
{
    /* Added by user */

    return USBD_OK;
}

/**
  * @brief  Get mouse device configuration descriptor
  * @param  USBSpeed: current device speed
  * @param  len: pointer to data length
  * @retval Pointer to descriptor buffer
  */
static uint8_t*  USBD_PRINTER_GetCfgDesc (uint8_t USBSpeed, uint16_t *len)
{
    *len = sizeof (USBD_PRINTER_CfgDesc);

    return (uint8_t *)USBD_PRINTER_CfgDesc;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
