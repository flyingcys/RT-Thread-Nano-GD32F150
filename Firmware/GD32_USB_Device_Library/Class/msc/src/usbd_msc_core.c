/**
  ******************************************************************************
  * @file    usbd_msc_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file provides all the MSC core functions.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_core.h"

/* Private function prototypes -----------------------------------------------*/
static uint8_t  USBD_MSC_Init               (void *pudev, uint8_t cfgidx);
static uint8_t  USBD_MSC_DeInit             (void *pudev, uint8_t cfgidx);
static uint8_t  USBD_MSC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_MSC_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_MSC_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_MSC_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_MSC_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_MSC_DataOut            (void *pudev, uint8_t EpID);
static uint8_t *USBD_MSC_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);
static uint8_t  USBD_MSC_ClearFeature       (void *pudev, USB_DEVICE_REQ *req);

USBD_Class_cb_TypeDef  USBD_MSC_cb = 
{
    USBD_MSC_Init,
    USBD_MSC_DeInit,
    USBD_MSC_GetClassDescriptor,
    USBD_MSC_ClassReqHandle,
    USBD_MSC_GetInterface,
    USBD_MSC_SetInterface,
    NULL, /* EP0_TxSent */
    NULL, /* EP0_RxReady */
    USBD_MSC_DataIn,
    USBD_MSC_DataOut,
    NULL, /* SOF */
    USBD_MSC_GetCfgDesc,
    USBD_MSC_ClearFeature
};

/* USB Mass storage device configuration descriptor set */
const uint8_t USBD_MSC_CfgDesc[USB_MSC_CONFIG_DESC_SIZE] =
{
    0x09,   /* bLength: configuration descriptor size */
    USB_DESCTYPE_CONFIGURATION, /* bDescriptorType: configuration descriptor type */
    USB_MSC_CONFIG_DESC_SIZE,   /* wTotalLength: configuration descriptor set total length */
    0x00,
    0x01,   /* bNumInterfaces: 1 interface */
    0x01,   /* bConfigurationValue: configuration value */
    0x04,   /* iConfiguration: index of string descriptor describing the configuration */
    0xC0,   /* bmAttributes: device attributes (bus powered and not support remote wakeup) */
    0x32,   /* MaxPower 100 mA: this current is used for detecting Vbus */

    /******************** interface descriptor ********************/
    0x09,   /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE, /* bDescriptorType: interface descriptor type*/
    0x00,   /* bInterfaceNumber: number of interface */
    0x00,   /* bAlternateSetting: alternate setting */
    0x02,   /* bNumEndpoints: use 2 endpoints for Tx and Rx */
    0x08,   /* bInterfaceClass: MSC Class */
    0x06,   /* bInterfaceSubClass: SCSI transparent*/
    0x50,   /* nInterfaceProtocol: Bulk-only transport */
    0x05,   /* iInterface: index of interface string descriptor */

    /********************  Mass storage endpoint descriptor ********************/
    0x07,       /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT,         /* bDescriptorType: endpoint descriptor type */
    MSC_IN_EP,  /* bEndpointAddress: endpoint address (EP1_IN) */
    0x02,       /* bmAttributes: endpoint attribute(bulk endpoint) */
    LOWBYTE(MSC_DATA_PACKET_SIZE), /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(MSC_DATA_PACKET_SIZE),
    0x00,       /* bInterval: polling interval is ignored */

    0x07,       /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT,         /* bDescriptorType: endpoint descriptor type */
    MSC_OUT_EP, /*bEndpointAddress: endpoint address (EP2_OUT) */
    0x02,       /* bmAttributes: endpoint attribute(bulk endpoint) */
    LOWBYTE(MSC_DATA_PACKET_SIZE), /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(MSC_DATA_PACKET_SIZE),
    0x00        /* bInterval: polling interval is ignored */
};

static uint8_t USBD_MSC_MaxLun = 0;
static uint8_t USBD_MSC_AltSet = 0;

/* Private function ----------------------------------------------------------*/
/**
  * @brief  Initialize the mass storage configuration
  * @param  pudev: pointer to usb device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
static uint8_t  USBD_MSC_Init (void *pudev, uint8_t cfgidx)
{
#ifdef DOUBLE_BUFFER_TX
    USB_EP_BufConfig(pudev, MSC_IN_EP, USB_DBL_BUFTYPE, MSC_TX_ADDRESS);
#else
    USB_EP_BufConfig(pudev, MSC_IN_EP, USB_SNG_BUFTYPE, MSC_TX_ADDRESS);
#endif

#ifdef DOUBLE_BUFFER_RX
    USB_EP_BufConfig(pudev, MSC_OUT_EP, USB_DBL_BUFTYPE, MSC_RX_ADDRESS);
#else
    USB_EP_BufConfig(pudev, MSC_OUT_EP, USB_SNG_BUFTYPE, MSC_RX_ADDRESS);
#endif

    /* Open EP IN */
    USB_EP_Init(pudev,
                MSC_IN_EP,
                USB_EPTYPE_BULK,
                MSC_EPIN_SIZE);
  
    /* Open EP OUT */
    USB_EP_Init(pudev,
                MSC_OUT_EP,
                USB_EPTYPE_BULK,
                MSC_EPOUT_SIZE);

    /* Init the BBB layer */
    MSC_BBB_Init(pudev); 

    return USBD_OK;
}

/**
  * @brief  DeInitilaize the mass storage configuration
  * @param  pudev: pointer to usb device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
static uint8_t  USBD_MSC_DeInit (void *pudev, uint8_t cfgidx)
{
    /* Close MSC EPs */
    USB_EP_DeInit(pudev, MSC_IN_EP);
    USB_EP_DeInit(pudev, MSC_OUT_EP);

    /* Un Init the BBB layer */
    MSC_BBB_DeInit(pudev);

    return USBD_OK;
}

/**
  * @brief  Handle the MSC class-specific requests
  * @param  pudev: pointer to usb device instance
  * @param  req: device class-specific request
  * @retval usb device operation status
  */
static uint8_t  USBD_MSC_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    switch (req->bRequest)
    {
        case BBB_GET_MAX_LUN :
            if((req->wValue  == 0) && 
                (req->wLength == 1) &&
                ((req->bmRequestType & 0x80) == 0x80))
            {
                USBD_MSC_MaxLun = USBD_STORAGE_fops->GetMaxLun();
                if(USBD_MSC_MaxLun > 0)
                {
                    USB_CtlTx(pudev, &USBD_MSC_MaxLun, 1);
                }
                else
                {
                    USBD_EnumError(pudev, req);
                    return USBD_FAIL; 
                }
            }
            else
            {
                USBD_EnumError(pudev, req);
                return USBD_FAIL; 
            }
            break;

        case BBB_RESET :
            if((req->wValue  == 0) && 
                (req->wLength == 0) &&
                 ((req->bmRequestType & 0x80) != 0x80))
            {
                MSC_BBB_Reset(pudev);
            }
            else
            {
                USBD_EnumError(pudev, req);
                return USBD_FAIL; 
            }
            break;

        default:
            USBD_EnumError(pudev, req);
            return USBD_FAIL; 
    }

    return USBD_OK;
}

/**
  * @brief  Get MSC class descriptor(Report descriptor)
  * @param  pudev: pointer to usb device instance
  * @param  req: device request
  * @retval usb device operation status
  */
static uint8_t  USBD_MSC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_MSC_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USB_CtlTx (pudev, (uint8_t *)&USBD_MSC_AltSet, 1);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_MSC_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USBD_MSC_AltSet = (uint8_t)(req->wValue);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Clear Feature
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_MSC_ClearFeature (void *pudev, USB_DEVICE_REQ *req)
{
    MSC_BBB_CplClrFeature(pudev, (uint8_t)req->wIndex);

    return USBD_OK;
}

/**
  * @brief  Handle data IN Stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
static uint8_t  USBD_MSC_DataIn (void *pudev, uint8_t EpID)
{
    MSC_BBB_DataIn(pudev, EpID);

    return USBD_OK;
}

/**
  * @brief  Handle data OUT Stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier
  * @retval status
  */
static uint8_t  USBD_MSC_DataOut (void *pudev, uint8_t EpID)
{
    MSC_BBB_DataOut(pudev, EpID);

    return USBD_OK;
}

/**
  * @brief  Get configuration descriptor
  * @param  speed : current device speed
  * @param  len : pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MSC_GetCfgDesc (uint8_t speed, uint16_t *len)
{
    *len = sizeof (USBD_MSC_CfgDesc);

    return (uint8_t *)USBD_MSC_CfgDesc;
}

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
