/**
  ******************************************************************************
  * @file    usbd_cdc_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    26-Dec-2014
  * @brief   USB CDC class device core functions.                               
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_loopback_core.h"
#include "usbd_enum.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_CDC_Class
  * @{
  */
static uint8_t  USBD_CDC_Init               (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_CDC_DeInit             (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_CDC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CDC_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CDC_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CDC_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CDC_EP0_RxReady        (void *pudev);
static uint8_t  USBD_CDC_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_CDC_DataOut            (void *pudev, uint8_t EpID);
static uint8_t* USBD_CDC_GetCfgDesc         (uint8_t USBSpeed, uint16_t *len);

/** @defgroup USB_CDC_Class_Private_Variables
  * @{
  */
static __IO uint32_t  USBD_CDC_AltSet = 0;

uint8_t USB_DATA_Buffer[USB_CDC_DATA_PACKET_SIZE];
uint8_t USB_CMD_Buffer[USB_CDC_CMD_PACKET_SIZE];

extern uint8_t PacketSent;
extern uint8_t PacketReceive;

static uint32_t CDCCmd = 0xFF;

uint32_t ReceiveLength = 0;

LINE_CODING linecoding =
{
    115200, /* Baud rate     */
    0x00,   /* Stop bits - 1 */
    0x00,   /* Parity - none */
    0x08    /* Num of bits 8 */
};


/* USB CDC device class callbacks structure */
USBD_Class_cb_TypeDef  USBD_CDC_cb =
{
    USBD_CDC_Init,
    USBD_CDC_DeInit,
    USBD_CDC_GetClassDescriptor,
    USBD_CDC_ClassReqHandle,
    USBD_CDC_GetInterface,
    USBD_CDC_SetInterface,
    NULL,/* EP0_TxSent */
    USBD_CDC_EP0_RxReady,
    USBD_CDC_DataIn,
    USBD_CDC_DataOut,
    NULL,
    USBD_CDC_GetCfgDesc,
};

/* USB CDC device configuration descriptor set */
const uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZE] =
{
    /*Configuration Descriptor*/
    0x09,   /* bLength: configuration descriptor size */
    USB_DESCTYPE_CONFIGURATION,   /* bDescriptorType: configuration descriptor type */
    USB_CDC_CONFIG_DESC_SIZE,     /* wTotalLength: configuration descriptor set total length */
    0x00,
    0x02,   /* bNumInterfaces: 2 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: index of string descriptor describing the configuration */
    0x80,   /* bmAttributes: device attributes (bus powered and not support remote wakeup) */
    0x32,   /* bMaxPower 100 mA: this current is used for detectiong Vbus */

    /*Interface descriptor */
    0x09,   /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE,  /* bDescriptorType: interface descriptor type */
    0x00,   /* bInterfaceNumber: number of interface */
    0x00,   /* bAlternateSetting: alternate setting */
    0x01,   /* bNumEndpoints: 1 endpoint used for command IN */
    0x02,   /* bInterfaceClass: CDC class */
    0x02,   /* bInterfaceSubClass: abstract control model */
    0x01,   /* nInterfaceProtocol: common AT commands */
    0x00,   /* iInterface: index of interface string descriptor */

    /* Header functional descriptor */
    0x05,   /* bFunctionLength: the descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: header function descriptor */
    0x10,   /* bcdCDC: spec release number (CDC1.10) */
    0x01,

    /* Call management functional descriptor */
    0x05,   /* bFunctionLength: the descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0 is reset, D1 is ignored */
    0x01,   /* bDataInterface: 1 interface used for call management */

    /* ACM functional descriptor */
    0x04,   /* bFunctionLength: the dsecriptor length */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities: D1 */

    /* Union functional descriptor */
    0x05,   /* bFunctionLength: the descriptor length */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /* Command endpoint descriptor */
    0x07,                           /* bLength: endpoint descriptor size */
    USB_DESCTYPE_ENDPOINT,          /* bDescriptorType: endpoint descriptor type */
    CDC_CMD_EP,                     /* bEndpointAddress: endpoint address(EP2_IN) */
    0x03,                           /* bmAttributes: interrupt endpoint */
    LOWBYTE(USB_CDC_CMD_PACKET_SIZE),    /* wMaxPacketSize: 8 bytes max */
    HIGHBYTE(USB_CDC_CMD_PACKET_SIZE),
    0x0A,                           /* bInterval: polling interval(10ms) */

    /* Data class interface descriptor */
    0x09,   /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE,  /* bDescriptorType: interface descriptor type */
    0x01,   /* bInterfaceNumber: number of interface */
    0x00,   /* bAlternateSetting: alternate setting */
    0x02,   /* bNumEndpoints: 2 endpoints used */
    0x0A,   /* bInterfaceClass: CDC class */
    0x00,   /* bInterfaceSubClass: no set */
    0x00,   /* bInterfaceProtocol: no set */
    0x00,   /* iInterface: no set */

    /* Data OUT endpoint descriptor */
    0x07,                              /* bLength: endpoint descriptor size */
    USB_DESCTYPE_ENDPOINT,             /* bDescriptorType: endpoint descriptor type */
    CDC_DATA_OUT_EP,                   /* bEndpointAddress: endpoint address(EP3_OUT) */
    0x02,                              /* bmAttributes: bulk endpoint */
    LOWBYTE(CDC_DATA_OUT_PACKET_SIZE), /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(CDC_DATA_OUT_PACKET_SIZE),
    0x00,                              /* bInterval: ignore for Bulk transfer */

    /* Data IN endpoint descriptor */
    0x07,                              /* bLength: endpoint descriptor size */
    USB_DESCTYPE_ENDPOINT,             /* bDescriptorType: endpoint descriptor type */
    CDC_DATA_IN_EP,                    /* bEndpointAddress: endpoint address(EP1_IN) */
    0x02,                              /* bmAttributes: bulk endpoint */
    LOWBYTE(CDC_DATA_IN_PACKET_SIZE),  /* wMaxPacketSize: 64 bytes max */
    HIGHBYTE(CDC_DATA_IN_PACKET_SIZE),
    0x00                               /* bInterval: ignore for bulk transfer */
};

/**
  * @}
  */

/** @defgroup USB_CDC_Class_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the CDC interface
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_Init (void *pudev, uint8_t ConfigIndex)
{
    USB_EP_BufConfig(pudev, CDC_DATA_IN_EP, USB_SNG_BUFTYPE, BULK_TX_ADDRESS);
    USB_EP_BufConfig(pudev, CDC_CMD_EP, USB_SNG_BUFTYPE, INT_TX_ADDRESS);
    USB_EP_BufConfig(pudev, CDC_DATA_OUT_EP, USB_SNG_BUFTYPE, BULK_RX_ADDRESS);

    /* Initialize the data Tx/Rx endpoint */
    USB_EP_Init(pudev, CDC_DATA_IN_EP, USB_EPTYPE_BULK, CDC_DATA_IN_PACKET_SIZE);
    USB_EP_Init(pudev, CDC_DATA_OUT_EP, USB_EPTYPE_BULK, CDC_DATA_OUT_PACKET_SIZE);

    /* Initialize the command Tx endpoint */
    USB_EP_Init(pudev, CDC_CMD_EP, USB_EPTYPE_INT, USB_CDC_CMD_PACKET_SIZE);

    return USBD_OK;
}

/**
  * @brief  De-Initialize the CDC interface
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_DeInit (void *pudev, uint8_t ConfigIndex)
{
    /* Deinitialize the data Tx/Rx endpoint */
    USB_EP_DeInit(pudev, CDC_DATA_IN_EP);
    USB_EP_DeInit(pudev, CDC_DATA_OUT_EP);

    /* Deinitialize the command Tx endpoint */
    USB_EP_DeInit(pudev, CDC_CMD_EP);

    return USBD_OK;
}

/**
  * @brief  Get CDC class descriptor
  * @param  pudev: pointer to usb device instance
  * @param  req: device request
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    uint16_t len = USB_CDC_DESC_SIZE;
    uint8_t  *pbuf= (uint8_t*)USBD_CDC_CfgDesc + 9;

    if((req->wValue >> 8) == CDC_DESC_TYPE)
    {
        len = MIN(USB_CDC_DESC_SIZE, req->wLength);
        pbuf = (uint8_t*)USBD_CDC_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);
    }

    USB_CtlTx (pudev, pbuf, len);

    return USBD_OK;
}

/**
  * @brief  Handle the CDC class-specific request
  * @param  pudev: pointer to usb device instance
  * @param  req: device class-specific request
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    switch (req->bRequest)
    {
        case SEND_ENCAPSULATED_COMMAND:
            /* No operation for this driver */
            break;

        case GET_ENCAPSULATED_RESPONSE:
            /* No operation for this driver */
            break;

        case SET_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case GET_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case CLEAR_COMM_FEATURE:
            /* No operation for this driver */
            break;

        case SET_LINE_CODING:
            /* Set the value of the current command to be processed */
            CDCCmd = req->bRequest;

            /* Enable EP0 prepare to receive command data packet */
            USB_CtlRx (pudev, USB_CMD_Buffer, req->wLength);
            break;

        case GET_LINE_CODING:
            USB_CMD_Buffer[0] = (uint8_t)(linecoding.dwDTERate);
            USB_CMD_Buffer[1] = (uint8_t)(linecoding.dwDTERate >> 8);
            USB_CMD_Buffer[2] = (uint8_t)(linecoding.dwDTERate >> 16);
            USB_CMD_Buffer[3] = (uint8_t)(linecoding.dwDTERate >> 24);
            USB_CMD_Buffer[4] = linecoding.bCharFormat;
            USB_CMD_Buffer[5] = linecoding.bParityType;
            USB_CMD_Buffer[6] = linecoding.bDataBits;

            /* Send the request data to the host */
            USB_CtlTx (pudev, USB_CMD_Buffer, req->wLength);
            break;

        case SET_CONTROL_LINE_STATE:
            /* No operation for this driver */
            break;

        case SEND_BREAK:
            /* No operation for this driver */
            break;

        default:
            break;
    }

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USB_CtlTx (pudev, (uint8_t *)&USBD_CDC_AltSet, 1);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
    {
        USBD_CDC_AltSet = (uint8_t)(req->wValue);
    }
    else
    {
        /* Call the error management function (command will be nacked */
        USBD_EnumError (pudev, req);
    }

    return USBD_OK;
}

/**
  * @brief  Command data received on control endpoint
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_EP0_RxReady (void  *pudev)
{
    if (CDCCmd != NO_CMD)
    {
        /* Process the command data */
        linecoding.dwDTERate = (uint32_t)(USB_CMD_Buffer[0] | 
                                         (USB_CMD_Buffer[1] << 8) | 
                                         (USB_CMD_Buffer[2] << 16) | 
                                         (USB_CMD_Buffer[3] << 24));

        linecoding.bCharFormat = USB_CMD_Buffer[4];
        linecoding.bParityType = USB_CMD_Buffer[5];
        linecoding.bDataBits = USB_CMD_Buffer[6];

        CDCCmd = NO_CMD;
    }

    return USBD_OK;
}

/**
  * @brief  Data sent on non-control IN endpoint
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_DataIn (void *pudev, uint8_t EpID)
{
    PacketSent = 1;

    return USBD_OK;
}

/**
  * @brief  Data received on non-control Out endpoint
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
static uint8_t  USBD_CDC_DataOut (void *pudev, uint8_t EpID)
{
    PacketReceive = 1;

    ReceiveLength = _GetEPRxCount(EpID);

    return USBD_OK;
}

/**
  * @brief  Getting configuration descriptor
  * @param  USBSpeed: current device speed
  * @param  len: pointer to data length 
  * @retval Pointer to configuration descriptor buffer
  */
static uint8_t  *USBD_CDC_GetCfgDesc (uint8_t USBSpeed, uint16_t *len)
{
    *len = sizeof (USBD_CDC_CfgDesc);

    return (uint8_t*)USBD_CDC_CfgDesc;
}

/**
  * @brief  Send the data received from the GD32 to the PC through USB
  * @param  None
  * @retval None
  */
void USBD_CDC_SendData (void *pudev, uint32_t DataLen)
{
    /* Limit the transfer data length */
    if(DataLen <= USB_CDC_DATA_PACKET_SIZE)
    {
        PacketSent = 0;

        USB_EP_Tx(pudev, CDC_DATA_IN_EP, (uint8_t*)(USB_DATA_Buffer), DataLen);
    }
}

/**
  * @brief  Receive the data from the PC to GD32 and send it through USB
  * @param  None
  * @retval None
  */
void USBD_CDC_ReceiveData(void *pudev)
{
    PacketReceive = 0;

    USB_EP_Rx(pudev, CDC_DATA_OUT_EP, (uint8_t*)(USB_DATA_Buffer), CDC_DATA_OUT_PACKET_SIZE);
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
