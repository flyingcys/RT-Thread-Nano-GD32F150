/**
  ******************************************************************************
  * @file    usbd_printer_cdc_wrapper.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file calls to the separate CDC and HID class layer handlers.
  *******************************************************************************/ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_printer_cdc_wrapper.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t  USBD_PRINTER_CDC_Init               (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_PRINTER_CDC_DeInit             (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_PRINTER_CDC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_CDC_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_CDC_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_CDC_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_PRINTER_CDC_EP0_RxReady        (void *pudev);
static uint8_t  USBD_PRINTER_CDC_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_PRINTER_CDC_DataOut            (void *pudev, uint8_t EpID);
static uint8_t  USBD_PRINTER_CDC_SOF                (void *pudev); 
static uint8_t* USBD_PRINTER_CDC_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);

/* HID_CDC_CORE_Private_Variables */
USBD_Class_cb_TypeDef  USBD_PRINTER_CDC_cb = 
{
    USBD_PRINTER_CDC_Init,
    USBD_PRINTER_CDC_DeInit,
    USBD_PRINTER_CDC_GetClassDescriptor,
    USBD_PRINTER_CDC_ClassReqHandle,
    USBD_PRINTER_CDC_GetInterface,
    USBD_PRINTER_CDC_SetInterface,
    NULL, /* EP0_TxSent */
    USBD_PRINTER_CDC_EP0_RxReady,
    USBD_PRINTER_CDC_DataIn,
    USBD_PRINTER_CDC_DataOut,
    USBD_PRINTER_CDC_SOF,
    USBD_PRINTER_CDC_GetCfgDesc,
    NULL
};

/* USB CDC_PRINTER device Configuration Descriptor */
const uint8_t USBD_PRINTER_CDC_CfgDesc[USB_PRINTER_CDC_CONFIG_DESC_SIZE] =
{
    0x09,         /* bLength: Configuration Descriptor size */
    USB_DESCTYPE_CONFIGURATION,  /* bDescriptorType: Configuration */
    USB_PRINTER_CDC_CONFIG_DESC_SIZE,/* wTotalLength: configuration descriptor set total length */
    0x00,
    0x03,         /* bNumInterfaces: 3 interfaces (2 for CDC, 1 for PRINTER) */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing the configuration */
    0xE0,         /* bmAttributes: bus powered and Support Remote Wake-up */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

    /************** interface descriptor  ****************/
    0x09,         /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE,/* bDescriptorType: interface descriptor type */
    PRINTER_INTERFACE,         /* bInterfaceNumber: number of interface */
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
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    PRINTER_OUT_EP,        /* bEndpointAddress: endpoint address (EP1_OUT) */
    0x02,                  /* bmAttributes: endpoint attribute(bulk endpoint) */
    PRINTER_OUT_PACKET,    /* wMaxPacketSize: 64 bytes max */
    0x00,
    0x00,                  /* bInterval: polling interval */

    /******** IAD should be positioned just before the CDC interfaces ******
                IAD to associate the two CDC interfaces */
    0x08, /* bLength */
    0x0B, /* bDescriptorType */
    0x01, /* bFirstInterface */
    0x02, /* bInterfaceCount */
    0x02, /* bFunctionClass */
    0x02, /* bFunctionSubClass */
    0x01, /* bFunctionProtocol */
    0x00, /* iFunction (Index of string descriptor describing this function) */

    /*************************** CDC interfaces *******************************/

    /*Interface Descriptor */
    0x09,   /* bLength: Interface Descriptor size */
    USB_DESCTYPE_INTERFACE, /* bDescriptorType:  Interface descriptor type */
    CDC_COM_INTERFACE,      /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x01,   /* iInterface: index of interface string descriptor */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0 is reset, D1 is ignored  */
    0x02,   /* bDataInterface: 2 interface used for call management */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities: D1 */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x01,   /* bMasterInterface: Communication class interface */
    0x02,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                             /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT,            /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                       /* bEndpointAddress: endpoint address(EP2_IN) */
    0x03,                             /* bmAttributes: Interrupt */
    LOWBYTE(USB_CDC_CMD_PACKET_SIZE), /* wMaxPacketSize: 8 bytes max */
    HIGHBYTE(USB_CDC_CMD_PACKET_SIZE),
    0xFF,                             /* bInterval: polling interval(255ms) */

    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_INTERFACE,  /* bDescriptorType: */
    0x02,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: 2 endpoints used */
    0x0A,   /* bInterfaceClass: CDC class */
    0x00,   /* bInterfaceSubClass: no set*/
    0x00,   /* bInterfaceProtocol: no set */
    0x00,   /* iInterface: no set */

    /*Endpoint OUT Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT,  /* bDescriptorType: endpoint descriptor type */
    CDC_DATA_OUT_EP,        /* bEndpointAddress: endpoint address */
    0x02,                   /* bmAttributes: Bulk */
    0x40,                   /* wMaxPacketSize: 64 bytes max */
    0x00,
    0x00,                   /* bInterval: ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT,  /* bDescriptorType: endpoint descriptor type */
    CDC_DATA_IN_EP,         /* bEndpointAddress: endpoint address */
    0x02,                   /* bmAttributes: Bulk */
    0x40,                   /* wMaxPacketSize: 64 bytes max */
    0x00,
    0x00,                   /* bInterval: ignore for Bulk transfer */ 
};

/* Private function prototypes -----------------------------------------------*/
extern USBD_Class_cb_TypeDef  USBD_Printer_cb;
extern USBD_Class_cb_TypeDef  USBD_CDC_cb;

/* Private function ----------------------------------------------------------*/

/**
  * @brief  Initialize the PRINTER & CDC interfaces
  * @param  pudev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_PRINTER_CDC_Init (void *pudev, uint8_t ConfigIndex)
{
    /* HID initialization */
    USBD_Printer_cb.Init(pudev, ConfigIndex);

    /* CDC initialization */
    USBD_CDC_cb.Init(pudev, ConfigIndex);

    return USBD_OK;
}

/**
  * @brief  DeInitialize the PRINTER/CDC interfaces
  * @param  pudev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_PRINTER_CDC_DeInit (void *pudev, uint8_t ConfigIndex)
{
    /* PRINTER De-initialization */
    USBD_Printer_cb.DeInit(pudev, ConfigIndex);

    /* CDC De-initialization */
    USBD_CDC_cb.DeInit(pudev, ConfigIndex);

    return USBD_OK;
}

/**
  * @brief  Get custom PRINTER/CDC class special descriptor(Report descriptor/CDC descriptor)
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_CDC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == PRINTER_INTERFACE)
    {
        return USBD_Printer_cb.GetClassDescriptor(pudev, req);
    }
    else
    {
        return USBD_CDC_cb.GetClassDescriptor(pudev, req);
    }
}


/**
  * @brief  Handle the custom PRINTER/CDC class-specific request
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_CDC_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == PRINTER_INTERFACE)
    {
        return USBD_Printer_cb.ClassReqHandle(pudev, req);
    }
    else
    {
        return USBD_CDC_cb.ClassReqHandle(pudev, req);
    }
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_CDC_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == PRINTER_INTERFACE)
    {
        return USBD_Printer_cb.GetInterface(pudev, req);
    }
    else
    {
        return USBD_CDC_cb.GetInterface(pudev, req);
    }
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_PRINTER_CDC_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == PRINTER_INTERFACE)
    {
        return USBD_Printer_cb.SetInterface(pudev, req);
    }
    else
    {
        return USBD_CDC_cb.SetInterface(pudev, req);
    }
}

/**
  * @brief  return configuration descriptor
  * @param  speed: current device speed
  * @param  len: pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t* USBD_PRINTER_CDC_GetCfgDesc (uint8_t USBSpeed, uint16_t *len)
{
    *len = sizeof (USBD_PRINTER_CDC_CfgDesc);

    return (uint8_t*)USBD_PRINTER_CDC_CfgDesc;
}

/**
  * @brief  Handle data IN Stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
static uint8_t  USBD_PRINTER_CDC_DataIn (void *pudev, uint8_t EpID)
{
    /*DataIN can be for CDC or HID */
    if (EpID == (CDC_DATA_IN_EP & ~0x80))
    {
        return USBD_CDC_cb.DataIn(pudev, EpID);
    }
    else
    {
        return USBD_Printer_cb.DataIn(pudev, EpID);
    }
}

/**
  * @brief  Handle data OUT Stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
uint8_t  USBD_PRINTER_CDC_DataOut(void *pudev , uint8_t EpID)
{
    /*DataOut can be for CDC or HID */
    if (EpID == (CDC_DATA_OUT_EP & ~0x80))
    {
        return USBD_CDC_cb.DataOut(pudev, EpID);
    }
    else
    {
        return USBD_Printer_cb.DataOut(pudev, EpID);
    }
}

/**
  * @brief  Handle SOF processing
  * @param  pudev: pointer to usb device instance
  * @retval status
  */
uint8_t  USBD_PRINTER_CDC_SOF (void *pudev)
{
    /*SOF processing needed for CDC */
    return USBD_CDC_cb.SOF(pudev);
}

/**
  * @brief  Handle RxReady processing
  * @param  pudev: pointer to usb device instance
  * @retval status
  */
static uint8_t  USBD_PRINTER_CDC_EP0_RxReady (void *pudev)
{
    /* RxReady processing needed for CDC */
    USBD_CDC_cb.EP0_RxReady(pudev);

    return USBD_OK;
}

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
