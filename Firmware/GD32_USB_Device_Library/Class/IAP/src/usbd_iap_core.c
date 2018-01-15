/**
  ******************************************************************************
  * @file    usbd_iap_core.c
  * @author  MCU SD
  * @version V1.0
  * @date    6-Sep-2014
  * @brief   Custom HID device class core functions for usb iap
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_iap_core.h"
#include "usbd_iap_mal.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_HID_Class
  * @{
  */
static uint8_t  USBD_CUSTOMHID_Init               (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_CUSTOMHID_DeInit             (void *pudev, uint8_t ConfigIndex);
static uint8_t  USBD_CUSTOMHID_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CUSTOMHID_ClassReqHandle     (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CUSTOMHID_GetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CUSTOMHID_SetInterface       (void *pudev, USB_DEVICE_REQ *req);
static uint8_t  USBD_CUSTOMHID_DataIn             (void *pudev, uint8_t EpID);
static uint8_t  USBD_CUSTOMHID_DataOut            (void *pudev, uint8_t EpID);
static uint8_t* USBD_CUSTOMHID_GetCfgDesc         (uint8_t Speed, uint16_t *len);

/* DFU Requests management functions */
static void  IAP_Req_ERASE     (void *pudev);
static void  IAP_Req_DNLOAD    (void *pudev);
static void  IAP_Req_OptionByte(void *pudev);
static void  IAP_Req_Leave     (void *pudev);
static void  IAP_SendAddress   (void *pudev);

/** @defgroup USB_CustomHID_Private_Variables
  * @{
  */
uint8_t Report_buf[65];
uint8_t OptionByte[17] = {0};

/* State machine variables */
uint8_t DeviceStatus[17];
uint8_t BinAddress[17];

uint8_t USBD_CUSTOMHID_Report_ID = 0;
uint8_t flag = 0;

static uint32_t  USBD_CUSTOMHID_AltSet = 0;
static uint32_t  USBD_CUSTOMHID_Protocol = 0;
static uint32_t  USBD_CUSTOMHID_IdleState = 0;

static uint16_t TransferTimes = 0;
static uint16_t PageCount = 0;
static uint32_t FileLength = 0;
static uint32_t BaseAddress = APP_LOADED_ADDR;

extern uint8_t  MAL_Buffer[TRANSFER_SIZE];

USBD_Class_cb_TypeDef  USBD_CUSTOMHID_cb = 
{
    USBD_CUSTOMHID_Init,
    USBD_CUSTOMHID_DeInit,
    USBD_CUSTOMHID_GetClassDescriptor,
    USBD_CUSTOMHID_ClassReqHandle,
    USBD_CUSTOMHID_GetInterface,
    USBD_CUSTOMHID_SetInterface,
    NULL, /* EP0_TxSent */
    NULL, /* EP0_RxReady */
    USBD_CUSTOMHID_DataIn,
    USBD_CUSTOMHID_DataOut,
    NULL, /* SOF */
    USBD_CUSTOMHID_GetCfgDesc,
};

/* USB HID device Configuration Descriptor */
const uint8_t USBD_HID_CfgDesc[CUSTOMHID_CONFIG_DESC_SIZE] =
{
    0x09,         /* bLength: Configuration Descriptor size */
    USB_DESCTYPE_CONFIGURATION, /* bDescriptorType: Configuration */
    CUSTOMHID_CONFIG_DESC_SIZE, /* wTotalLength: configuration descriptor set total length */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: configuration value */
    0x00,         /* iConfiguration: index of string descriptor describing the configuration */
    0x80,         /* bmAttributes: device attributes (bus powered and not support remote wakeup) */
    0x32,         /* bMaxPower 100 mA: this current is used for detecting Vbus */

    /************** Custom HID interface descriptor ****************/
    0x09,         /* bLength: interface descriptor size */
    USB_DESCTYPE_INTERFACE,/* bDescriptorType: interface descriptor type */
    0x00,         /* bInterfaceNumber: number of interface */
    0x00,         /* bAlternateSetting: alternate setting */
    0x02,         /* bNumEndpoints: 2 endpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass: 1 = BIOS boot, 0 = no boot */
    0x00,         /* nInterfaceProtocol: 0 = none, 1 = keyboard, 2 = mouse */
    0x00,         /* iInterface: index of string descriptor */

    /******************** HID descriptor ********************/
    0x09,         /* bLength: HID descriptor size */
    CUSTOMHID_DESC_TYPE, /* bDescriptorType: HID */
    0x11,         /* bcdHID: HID class protocol(HID1.11) */
    0x01,
    0x00,         /* bCountryCode: device country code */
    0x01,         /* bNumDescriptors: number of HID class descriptors to follow */
    0x22,         /* bDescriptorType: followed class descriptor type(report descriptor) */
    CUSTOMHID_REPORT_DESC_SIZE, /* wDescriptorLength: total length of report descriptor */
    0x00,

    /******************** Custom HID endpoints descriptor ***********/
    0x07,                  /* bLength: Endpoint Descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    CUSTOMHID_IN_EP,       /* bEndpointAddress: endpoint address (IN) */
    0x03,                  /* bmAttributes: endpoint attribute(interrupt endpoint) */
    CUSTOMHID_IN_PACKET,   /* wMaxPacketSize: 2 bytes max */
    0x00,
    0x20,                  /* bInterval: Polling Interval (32 ms) */

    0x07,                  /* bLength: endpoint descriptor size */
    USB_DESCTYPE_ENDPOINT, /* bDescriptorType: endpoint descriptor type */
    CUSTOMHID_OUT_EP,      /* bEndpointAddress: endpoint address (OUT) */
    0x03,                  /* bmAttributes: endpoint attribute(interrupt endpoint) */
    CUSTOMHID_OUT_PACKET,  /* wMaxPacketSize: 2 Bytes max */
    0x00,
    0x20,                  /* bInterval: Polling Interval (32 ms) */
};

/* USB custom HID device report descriptor */
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_REPORT_DESC_SIZE] =
{
    0x05, 0x01,     /* USAGE_PAGE (Generic Desktop) */
    0x09, 0x00,     /* USAGE (Custom Device)        */
    0xa1, 0x01,     /* COLLECTION (Application)     */

    /* IAP command and data */
    0x85, 0x01,     /* REPORT_ID (0x01)          */
    0x09, 0x01,     /* USAGE (IAP command)       */
    0x15, 0x00,     /* LOGICAL_MINIMUM (0)       */
    0x25, 0xff,     /* LOGICAL_MAXIMUM (255)     */
    0x75, 0x08,     /* REPORT_SIZE (8)           */
    0x95, 0x3f,     /* REPORT_COUNT (63)         */
    0x91, 0x82,     /* OUTPUT (Data,Var,Abs,Vol) */

    /* Device Status and Device Option Byte */  
    0x85, 0x02,     /* REPORT_ID (0x02)               */
    0x09, 0x02,     /* USAGE (Status and option byte) */
    0x15, 0x00,     /* LOGICAL_MINIMUM (0)            */
    0x25, 0xff,     /* LOGICAL_MAXIMUM (255)          */
    0x75, 0x08,     /* REPORT_SIZE (8)                */
    0x95, 0x10,     /* REPORT_COUNT (16)              */
    0x81, 0x82,     /* INPUT (Data,Var,Abs,Vol)       */

    0xc0            /* END_COLLECTION            */
}; 

/**
  * @}
  */

/** @defgroup USB_CustomHID_Handle_Function
  * @{
  */

/**
  * @brief  Initialize the custom HID device
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_Init (void *pudev, uint8_t ConfigIndex)
{
    USB_EP_BufConfig(pudev, CUSTOMHID_IN_EP, USB_SNG_BUFTYPE, HID_TX_ADDRESS);
    USB_EP_BufConfig(pudev, CUSTOMHID_OUT_EP, USB_SNG_BUFTYPE, HID_RX_ADDRESS);

    /* Initialize Tx endpoint */
    USB_EP_Init(pudev,
                CUSTOMHID_IN_EP,
                USB_EPTYPE_INT,
                CUSTOMHID_IN_PACKET);

    /* Initialize Rx endpoint */
    USB_EP_Init(pudev,
                CUSTOMHID_OUT_EP,
                USB_EPTYPE_INT,
                CUSTOMHID_OUT_PACKET);

    /* Initialize the MAL(Media Access Layer) */
    IAP_MAL_Init();

    /* Prepare receive Data */
    USB_EP_Rx(pudev, CUSTOMHID_OUT_EP, Report_buf, 64);

    return USBD_OK;
}

/**
  * @brief  De-Initialize the custom HID device
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_DeInit (void *pudev, uint8_t ConfigIndex)
{
    /* Deinitialize the custom HID endpoints */
    USB_EP_DeInit (pudev, CUSTOMHID_IN_EP);
    USB_EP_DeInit (pudev, CUSTOMHID_OUT_EP);

    /* Deinitilialize the MAL(Media Access Layer) */
    IAP_MAL_DeInit();

    return USBD_OK;
}

/**
  * @brief  Handle the custom HID class-specific request
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_ClassReqHandle (void *pudev, USB_DEVICE_REQ *req)
{
    uint8_t USBD_CUSTOMHID_Report_LENGTH = 0;

    switch (req->bRequest)
    {
        case GET_REPORT:
            /* No use for this driver */
            break;

        case GET_IDLE:
            USB_CtlTx (pudev, (uint8_t *)&USBD_CUSTOMHID_IdleState, 1);
            break;

        case GET_PROTOCOL:
            USB_CtlTx (pudev, (uint8_t *)&USBD_CUSTOMHID_Protocol, 1);
            break;

        case SET_REPORT:
            flag = 1;
            USBD_CUSTOMHID_Report_ID = (uint8_t)(req->wValue);
            USBD_CUSTOMHID_Report_LENGTH = (uint8_t)(req->wLength);
            USB_CtlRx (pudev, Report_buf, USBD_CUSTOMHID_Report_LENGTH);
            break;

        case SET_IDLE:
            USBD_CUSTOMHID_IdleState = (uint8_t)(req->wValue >> 8);
            break;

        case SET_PROTOCOL:
            USBD_CUSTOMHID_Protocol = (uint8_t)(req->wValue);
            break;

        default:
            USBD_EnumError (pudev, req);
            return USBD_FAIL; 
    }

    return USBD_OK;
}

/**
  * @brief  Get custom HID class special descriptor(Report descriptor)
  * @param  pudev: pointer to usb device instance
  * @param  req: device class request
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_GetClassDescriptor (void *pudev, USB_DEVICE_REQ *req)
{
    uint16_t len = 0;
    uint8_t  *pbuf = NULL;

    switch(req->wValue >> 8)
    {
        case CUSTOMHID_REPORT_DESCTYPE:
            len = MIN(CUSTOMHID_REPORT_DESC_SIZE, req->wLength);
            pbuf = (uint8_t *)CustomHID_ReportDescriptor;
            break;

        case CUSTOMHID_DESC_TYPE:
            len = MIN(USB_CUSTOMHID_DESC_SIZE, req->wLength);
            pbuf = (uint8_t *)USBD_HID_CfgDesc + 0x12;
            break;

        default:
            break;
    }

    USB_CtlTx (pudev, pbuf, len);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Get Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_GetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USB_CtlTx (pudev, (uint8_t *)&USBD_CUSTOMHID_AltSet, 1);

    return USBD_OK;
}

/**
  * @brief  Handle standard device request--Set Interface
  * @param  pudev: pointer to usb device instance
  * @param  req: standard device request
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_SetInterface (void *pudev, USB_DEVICE_REQ *req)
{
    USBD_CUSTOMHID_AltSet = (uint8_t)(req->wValue);

    return USBD_OK;
}

/**
  * @brief  Send custom HID Report
  * @param  pudev: pointer to usb device instance
  * @param  report: pointer to report
  * @param  Len: data length
  * @retval usb device operation status
  */
uint8_t USBD_CUSTOMHID_SendReport (USB_DEVICE_HANDLE *pudev,
                                            uint8_t *report,
                                            uint16_t Len)
{
    if (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED)
    {
        USB_EP_Tx (pudev, CUSTOMHID_IN_EP, report, Len);
    }

    return USBD_OK;
}

/**
  * @brief  Get custom HID device configuration descriptor
  * @param  Speed: current device speed
  * @param  len: pointer to data length
  * @retval Pointer to descriptor buffer
  */
uint8_t  *USBD_CUSTOMHID_GetCfgDesc (uint8_t Speed, uint16_t *len)
{
    *len = sizeof (USBD_HID_CfgDesc);

    return (uint8_t*)USBD_HID_CfgDesc;
}

/**
  * @brief  Handle custom HID device data IN stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_DataIn (void *pudev, uint8_t EpID)
{
    return USBD_OK;
}

/**
  * @brief  Handle custom HID device data OUT stage
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
uint8_t  USBD_CUSTOMHID_DataOut (void *pudev, uint8_t EpID)
{
    if (EpID == 1) 
    {
        switch (Report_buf[0])
        {
            case 0x01:
                switch(Report_buf[1])
                {
                    case IAP_DNLOAD:
                        IAP_Req_DNLOAD(pudev);
                        break;

                    case IAP_ERASE:
                        IAP_Req_ERASE(pudev);
                        break;
                    
                    case IAP_OPTION_BYTE:
                        IAP_Req_OptionByte(pudev);
                        break;

                    case IAP_LEAVE:
                        IAP_Req_Leave(pudev);
                        break;

                    case IAP_GETBIN_ADDRESS:
                        IAP_SendAddress(pudev);
                        break;

                    default:
                        USBD_EnumError(pudev, NULL);
                        return USBD_FAIL;
                }
                break;

            default:
                break;
        }
    }

    USB_EP_Rx(pudev, CUSTOMHID_OUT_EP, Report_buf, 64);

    return USBD_OK;
}

/**
  * @brief  Handle the IAP_DNLOAD request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void IAP_Req_DNLOAD(void *pudev)
{
    if(TransferTimes != 0)
    {
        if(TransferTimes == 1)
        {
            IAP_MAL_Write(&Report_buf[2], BaseAddress, FileLength % TRANSFER_SIZE);

            DeviceStatus[0] = 0x02;
            DeviceStatus[1] = 0x02;
            USBD_CUSTOMHID_SendReport (pudev, DeviceStatus, 17);
        }
        else
        {
            IAP_MAL_Write(&Report_buf[2], BaseAddress, TRANSFER_SIZE);

            BaseAddress += TRANSFER_SIZE;
        }

        TransferTimes --;
    }
}

/**
  * @brief  Handle the IAP_ERASE request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void IAP_Req_ERASE(void *pudev)
{
    BaseAddress  = Report_buf[2];
    BaseAddress |= Report_buf[3] << 8;
    BaseAddress |= Report_buf[4] << 16;
    BaseAddress |= Report_buf[5] << 24;

    PageCount = Report_buf[6];

    FileLength = Report_buf[7];
    FileLength |= Report_buf[8] << 8;
    FileLength |= Report_buf[9] << 16;
    FileLength |= Report_buf[10] << 24;

    if((FileLength % TRANSFER_SIZE) == 0)
    {
        TransferTimes = FileLength / TRANSFER_SIZE;
    }
    else
    {
        TransferTimes = FileLength / TRANSFER_SIZE + 1;
    }

    IAP_MAL_Erase(BaseAddress, PageCount);

    DeviceStatus[0] = 0x02;
    DeviceStatus[1] = 0x01;

    USB_EP_Tx(pudev, CUSTOMHID_IN_EP, DeviceStatus, 17);
}

/**
  * @brief  Handle the IAP_OPTION_BYTE request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  IAP_Req_OptionByte(void *pudev)
{
    uint8_t i = 0;
    uint32_t Address = 0x1FFFF800;

    OptionByte[0]= 0x02;

    for(i = 1; i < 17; i++)
    {
        OptionByte[i] = *(uint8_t *)Address;
        Address++;
    }

    USBD_CUSTOMHID_SendReport (&USB_Device_dev, OptionByte, 17);
}

/**
  * @brief  Handle the IAP_LEAVE request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  IAP_Req_Leave(void *pudev)
{
    IAP_MAL_DeInit();

    /* Generate system reset to allow jumping to the user code */
    NVIC_SystemReset();
}

/**
  * @brief  Handle the IAP_GETBIN_ADDRESS request
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
static void  IAP_SendAddress(void *pudev)
{
    BinAddress[0] = 0x02;

    BinAddress[1] = (uint8_t)(APP_LOADED_ADDR);
    BinAddress[2] = (uint8_t)(APP_LOADED_ADDR >> 8);
    BinAddress[3] = (uint8_t)(APP_LOADED_ADDR >> 16);
    BinAddress[4] = (uint8_t)(APP_LOADED_ADDR >> 24);

    USBD_CUSTOMHID_SendReport (&USB_Device_dev, BinAddress, 17);
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
