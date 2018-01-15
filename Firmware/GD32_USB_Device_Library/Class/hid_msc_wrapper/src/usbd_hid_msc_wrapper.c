/**
  ******************************************************************************
  * @file    usbd_msc_hid_wrapper.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file calls to the separate MSC and HID class layer handlers.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_msc_wrapper.h"
#include "usbd_custom_hid_core.h"
#include "usbd_msc_core.h"
#include "usbd_desc.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t  USBD_HID_MSC_Init               (void *pudev, uint8_t cfgidx);
static uint8_t  USBD_HID_MSC_DeInit             (void *pudev, uint8_t cfgidx);
static uint8_t  USBD_HID_MSC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_HID_MSC_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_HID_MSC_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_HID_MSC_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_HID_MSC_EP0_RxReady        (void *pudev);
static uint8_t  USBD_HID_MSC_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_HID_MSC_DataOut            (void *pudev, uint8_t EpID);
static uint8_t* USBD_HID_MSC_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);
static uint8_t  USBD_HID_MSC_ClearFeature       (void *pudev, USB_DEVICE_REQ *req);

USBD_Class_cb_TypeDef  USBD_HID_MSC_cb = 
{
    USBD_HID_MSC_Init,
    USBD_HID_MSC_DeInit,
    USBD_HID_MSC_GetClassDescriptor,
    USBD_HID_MSC_ClassReqHandle,
    USBD_HID_MSC_GetInterface,
    USBD_HID_MSC_SetInterface,
    NULL,
    USBD_HID_MSC_EP0_RxReady, 
    USBD_HID_MSC_DataIn, 
    USBD_HID_MSC_DataOut,
    NULL,
    USBD_HID_MSC_GetCfgDesc,
    USBD_HID_MSC_ClearFeature
};


/* USB MSC_HID device Configuration Descriptor */
const uint8_t USBD_HID_MSC_CfgDesc[USB_HID_MSC_CONFIG_DESC_SIZE] =
{
    0x09,         /* bLength: Configuration Descriptor size */
    USB_DESCTYPE_CONFIGURATION,  /* bDescriptorType: configuration descriptor type */
    USB_HID_MSC_CONFIG_DESC_SIZE,/* wTotalLength: configuration descriptor set total length */
    0x00,
    0x02,         /* bNumInterfaces: 2 interfaces (1 for MSC, 1 for HID) */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing the configuration */
    0xE0,         /* bmAttributes: bus powered and Support Remote Wake-up */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

    /************** Descriptor of CUSTOM HID interface ****************/
    0x09,         /* bLength: Interface Descriptor size */
    USB_DESCTYPE_INTERFACE,/* bDescriptorType: Interface descriptor type */
    HID_INTERFACE,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x02,         /* bNumEndpoints: 2 endpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass: 1 = BIOS boot, 0 = no boot */
    0x00,         /* nInterfaceProtocol: 0 = none, 1 = keyboard, 2 = mouse */
    0x00,         /* iInterface: Index of string descriptor */

    /******************** Descriptor of CUSTOM HID  **************************/
    0x09,         /* bLength: HID Descriptor size */
    CUSTOMHID_DESC_TYPE, /* bDescriptorType: HID */
    0x11,         /* bcdHID: HID class protocol(HID1.11) */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType: followed class descriptor type(report descriptor) */
    CUSTOMHID_REPORT_DESC_SIZE,/* wDescriptorLength: Total length of Report descriptor */
    0x00,

    /******************** Custom HID endpoints descriptor *********************/
    0x07,                  /* bLength: Endpoint Descriptor size*/
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    CUSTOMHID_IN_EP,       /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                  /* bmAttributes: Interrupt endpoint */
    CUSTOMHID_IN_PACKET,   /* wMaxPacketSize: 2 Byte max */
    0x00,
    0x20,                  /*bInterval: Polling Interval (32 ms)*/

    0x07,                  /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: Endpoint descriptor type */
    CUSTOMHID_OUT_EP,      /* bEndpointAddress: Endpoint Address (OUT) */
    0x03,                  /* bmAttributes: Interrupt endpoint */
    CUSTOMHID_OUT_PACKET,  /* wMaxPacketSize: 2 Bytes max */
    0x00,
    0x20,                  /* bInterval: Polling Interval (32 ms) */

    /********************  Mass Storage interface ********************/
    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: interface descriptor type */
    MSC_INTERFACE,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: use 2 endpoints for Tx and Rx */
    0x08,   /* bInterfaceClass: MSC Class */
    0x06,   /* bInterfaceSubClass: SCSI transparent */
    0x50,   /* nInterfaceProtocol: Bulk-only transport */
    0x01,   /* iInterface: index of interface string descriptor */

    /********************  Mass Storage Endpoints ********************/
    0x07,      /* bLength: Endpoint descriptor length = 7 */
    0x05,      /* bDescriptorType: Endpoint descriptor type */
    MSC_IN_EP, /* bEndpointAddress: Endpoint address (IN, address 2) */
    0x02,      /* bmAttributes:Bulk endpoint type */
    LOWBYTE(MSC_DATA_PACKET_SIZE), /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(MSC_DATA_PACKET_SIZE),
    0x00,      /* bInterval: polling interval is ignored */

    0x07,      /* bLength: Endpoint descriptor length = 7 */
    0x05,      /* bDescriptorType: Endpoint descriptor type */
    MSC_OUT_EP,/* bEndpointAddress: Endpoint address (OUT, address 2) */
    0x02,      /* bmAttributes: endpoint attribute(bulk endpoint) */
    LOWBYTE(MSC_DATA_PACKET_SIZE), /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(MSC_DATA_PACKET_SIZE),
    0x00       /* bInterval: polling interval is ignored */
};

/* Private function prototypes -----------------------------------------------*/
extern USBD_Class_cb_TypeDef  USBD_CUSTOMHID_cb;
extern USBD_Class_cb_TypeDef  USBD_MSC_cb;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the HID & MSC interfaces
  * @param  pudev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_MSC_Init (void *pudev, uint8_t cfgidx)
{
    /* HID initialization */
    USBD_CUSTOMHID_cb.Init(pudev, cfgidx);

    /* MSC initialization */
    USBD_MSC_cb.Init(pudev, cfgidx);

    return USBD_OK;

}

/**
  * @brief  DeInitialize the HID/CDC interfaces
  * @param  pudev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_MSC_DeInit (void  *pudev, uint8_t cfgidx)
{
    /* HID De-initialization */
    USBD_CUSTOMHID_cb.DeInit(pudev, cfgidx);

    /* MSC De-initialization */
    USBD_MSC_cb.DeInit(pudev, cfgidx);

    return USBD_OK;
}

/**
  * @brief  Get custom HID/MSC class special descriptor(Report descriptor)
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
static uint8_t  USBD_HID_MSC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == HID_INTERFACE)
    {
        return USBD_CUSTOMHID_cb.GetClassDescriptor(pudev, req);
    }
    else
    {
        return USBD_MSC_cb.GetClassDescriptor(pudev, req);
    }
}

/**
  * @brief  Handle the custom HID/MSC class-specific request
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
static uint8_t  USBD_HID_MSC_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == HID_INTERFACE)
    {
        return USBD_CUSTOMHID_cb.ClassReqHandle(pudev, req);
    }
    else
    {
        return USBD_MSC_cb.ClassReqHandle(pudev, req);
    }
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_HID_MSC_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == HID_INTERFACE)
    {
        return USBD_CUSTOMHID_cb.GetInterface(pudev, req);
    }
    else
    {
        return USBD_MSC_cb.GetInterface(pudev, req);
    }
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_HID_MSC_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if (req->wIndex == HID_INTERFACE)
    {
        return USBD_CUSTOMHID_cb.SetInterface(pudev, req);
    }
    else
    {
        return USBD_MSC_cb.SetInterface(pudev, req);
    }
}

/**
  * @brief  Get configuration descriptor
  * @param  USBSpeed: current device speed
  * @param  len: pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t* USBD_HID_MSC_GetCfgDesc (uint8_t USBSpeed, uint16_t *len)
{
    *len = sizeof (USBD_HID_MSC_CfgDesc);

    return (uint8_t*)USBD_HID_MSC_CfgDesc;
}

/**
  * @brief  Handle standard device request--Clear feature
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_HID_MSC_ClearFeature (void *pudev, USB_DEVICE_REQ *req)
{
	return USBD_MSC_cb.ClearFeature(pudev, req);
}

/**
  * @brief  Handle data IN Stage
  * @param  pudev: device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
static uint8_t  USBD_HID_MSC_DataIn (void *pudev, uint8_t EpID)
{
    /*DataIN can be for MSC or HID */
    if (EpID == (MSC_IN_EP & ~0x80))
    {
        return USBD_MSC_cb.DataIn(pudev, EpID);
    }
    else
    {
        return USBD_CUSTOMHID_cb.DataIn(pudev, EpID);
    }
}

/**
  * @brief  handle data OUT Stage
  * @param  pudev: device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
static uint8_t  USBD_HID_MSC_DataOut(void *pudev , uint8_t EpID)
{
    /*DataOut can be for MSC or HID */
    if (EpID == (MSC_OUT_EP & ~0x80))
    {
        return USBD_MSC_cb.DataOut(pudev, EpID);
    }
    else
    {
        return USBD_CUSTOMHID_cb.DataOut(pudev, EpID);
    }
}

/**
  * @brief  handle RxReady processing
  * @param  pudev: device instance
  * @retval status
  */
static uint8_t  USBD_HID_MSC_EP0_RxReady (void *pudev)
{
    /*RxReady processing needed for Custom HID only*/
    return USBD_CUSTOMHID_cb.EP0_RxReady(pudev);
}

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
