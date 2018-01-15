/**
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   Device Driver Header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_USB_CORE_H
#define __GD32F1X0_USB_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usb_buf.h"
#include "usbd_def.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Core_Exported_defines
  * @{
  */

/**
  * @brief Interrupt flag mask which decide what event should be handled by application 
  */
#define IER_MASK (CTLR_STIE | USB_INTR_PMOU | USB_INTR_ERR | USB_INTR_WKUP | USB_INTR_SPS  | \
                  USB_INTR_SOF | USB_INTR_ESOF | CTLR_RSTIE)

/**
  * @brief USB endpoint type
  */
#define USB_EPTYPE_CONTROL                     0
#define USB_EPTYPE_ISOC                        1
#define USB_EPTYPE_BULK                        2
#define USB_EPTYPE_INT                         3

/**
  * @brief USB endpoint kind
  */
#define USB_SNG_BUFTYPE                        0
#define USB_DBL_BUFTYPE                        1

/**
  * @brief USB control transfer status
  */
#define USB_CTRL_IDLE                          0
#define USB_CTRL_SETUP                         1
#define USB_CTRL_DATA_IN                       2
#define USB_CTRL_DATA_OUT                      3
#define USB_CTRL_STATUS_IN                     4
#define USB_CTRL_STATUS_OUT                    5

/**
  * @brief USB endpoint0 max packet size
  */
#define USB_EP0_MAX_SIZE                       64

/**
  * @brief USB device status
  */
#define USB_STATUS_UNCONNECTED                 0
#define USB_STATUS_DEFAULT                     1
#define USB_STATUS_ADDRESSED                   2
#define USB_STATUS_CONFIGURED                  3
#define USB_STATUS_SUSPENDED                   4
#define USB_STATUS_CONNECTED                   5

/**
  * @}
  */

/** @defgroup USB_Core_Exported_types
  * @{
  */

/**
  * @brief USB operation status
  */
typedef enum
{
    USB_OK   = 0,
    USB_FAIL,
}
USB_Status;

/**
  * @brief USB endpoint struct
  */
typedef struct USB_ep
{
    /* Basic parameters */
    uint8_t        is_stall;
    uint8_t        is_dblbuf;
    uint16_t       bufaddress;
    uint16_t       buf0addr;
    uint16_t       buf1addr;
    uint32_t       maxpacket;
    /* Transaction level parameters */
    uint8_t       *xfer_buf;
    uint32_t       xfer_len;
    uint32_t       xfer_count;
    /* Just for endpoint0 */
    uint32_t       ctrl_count;
}
USB_EP;

/**
  * @brief USB standard device request struct
  */
typedef  struct  usb_device_req
{
    uint8_t        bmRequestType;
    uint8_t        bRequest;
    uint16_t       wValue;
    uint16_t       wIndex;
    uint16_t       wLength;
}
USB_DEVICE_REQ;

/**
  * @brief USB device descriptor callback type define
  */
typedef struct _Device_Desc_cb
{
    uint8_t  *(*GetDeviceDescriptor)            (uint8_t USBSpeed, uint16_t *len);
#ifdef LPM_ENABLED
    uint8_t  *(*GetBOSDescriptor)               (uint8_t USBSpeed, uint16_t *len);
#endif
    uint8_t  *(*GetLangIDStrDescriptor)         (uint8_t USBSpeed, uint16_t *len);
    uint8_t  *(*GetManufacturerStrDescriptor)   (uint8_t USBSpeed, uint16_t *len);
    uint8_t  *(*GetProductStrDescriptor)        (uint8_t USBSpeed, uint16_t *len);
    uint8_t  *(*GetSerialStrDescriptor)         (uint8_t USBSpeed, uint16_t *len);
    uint8_t  *(*GetConfigurationStrDescriptor)  (uint8_t USBSpeed, uint16_t *len);
    uint8_t  *(*GetInterfaceStrDescriptor)      (uint8_t USBSpeed, uint16_t *len);
}
USBD_Desc_cb_TypeDef;

/**
  * @brief USB device class callback type define
  */
typedef struct _Device_Class_cb
{
    uint8_t  (*Init)         (void *pudev, uint8_t ConfigIndex);
    uint8_t  (*DeInit)       (void *pudev, uint8_t ConfigIndex);
    /* Control endpoints handle */
    uint8_t  (*GetClassDescriptor)   (void *pudev, USB_DEVICE_REQ *req);
    uint8_t  (*ClassReqHandle)       (void *pudev, USB_DEVICE_REQ *req);
    uint8_t  (*GetInterface) (void *pudev, USB_DEVICE_REQ *req);
    uint8_t  (*SetInterface) (void *pudev, USB_DEVICE_REQ *req);
    uint8_t  (*EP0_TxSent)   (void *pudev);
    uint8_t  (*EP0_RxReady)  (void *pudev);
    /* Class specific endpoints handle */
    uint8_t  (*DataIn)       (void *pudev, uint8_t EpID);
    uint8_t  (*DataOut)      (void *pudev, uint8_t EpID);
    uint8_t  (*SOF)          (void *pudev);
    uint8_t  *(*GetConfigDescriptor)(uint8_t USBSpeed, uint16_t *len);
#ifdef USB_SUPPORT_USER_STRING_DESC
    uint8_t  *(*GetUsrStrDescriptor)(uint8_t USBSpeed, uint8_t index, uint16_t *len);
#endif
    uint8_t  (*ClearFeature)         (void *pudev, USB_DEVICE_REQ *req);
} 
USBD_Class_cb_TypeDef;

/**
  * @brief USB user callback type define
  */
typedef struct _USBD_USER_cb
{
    void (*DeviceInit)        (void);
    void (*DeviceReset)       (uint8_t USBSpeed);
    void (*DeviceConfigured)  (void);
    void (*DeviceSuspended)   (void);
    void (*DeviceResumed)     (void);
}
USBD_User_cb_TypeDef;

/**
  * @brief USB core driver type define
  */
typedef struct _UCD
{
    /* Basic parameters */
    uint8_t        speed;
    uint8_t        device_cur_config;
    uint8_t        device_cur_state;
    uint8_t        device_cur_status;
    uint8_t        device_old_status;
    uint8_t        DevRemoteWakeup;
    uint8_t        setup_packet[8];
    /* Device endpoints */
    USB_EP         in_ep[EP_COUNT];
    USB_EP         out_ep[EP_COUNT];
    /* Device handle callback class */
    USBD_Class_cb_TypeDef     *class_cb;
    USBD_User_cb_TypeDef      *user_cb;
    USBD_Desc_cb_TypeDef      *desc_cb;
}
UCD_DEV, *UCD_PDEV;

typedef struct USB_Device_handle
{
    UCD_DEV        dev;
}
USB_DEVICE_HANDLE, USB_CORE_HANDLE;

/**
  * @}
  */

/** @defgroup USB_Core_Exported_Functions
  * @{
  */
void  DR_Init       (void);
void  DR_StopDevice (void);

void  USB_EP_SetAddress (USB_CORE_HANDLE *pudev, uint8_t Addr);
void  USB_EP_BufConfig  (USB_CORE_HANDLE *pudev, uint8_t EpAddr, uint8_t EpKind, uint32_t BufAddr);
void  USB_EP_Init       (USB_CORE_HANDLE *pudev, uint8_t EpAddr, uint8_t EpType, uint16_t EpMps);
void  USB_EP_DeInit     (USB_CORE_HANDLE *pudev, uint8_t EpAddr);
void  USB_EP_Tx         (USB_CORE_HANDLE *pudev, uint8_t EpAddr, uint8_t *pbuf, uint16_t BufLen);
void  USB_EP_Rx         (USB_CORE_HANDLE *pudev, uint8_t EpAddr, uint8_t *pbuf, uint16_t BufLen);
void  USB_EP_Stall      (USB_CORE_HANDLE *pudev, uint8_t EpID);
void  USB_EP_ClrStall   (USB_CORE_HANDLE *pudev, uint8_t EpID);

uint8_t  USB_EP_GetStatus (USB_CORE_HANDLE *pudev, uint8_t EpID);

uint8_t  USB_CtlTx             (USB_CORE_HANDLE *pudev, uint8_t *pbuf, uint16_t Len);
uint8_t  USB_CtlContinueTx     (USB_CORE_HANDLE *pudev, uint8_t *pbuf, uint16_t Len);
uint8_t  USB_CtlRx             (USB_CORE_HANDLE *pudev, uint8_t *pbuf, uint16_t Len);
uint8_t  USB_CtlContinueRx     (USB_CORE_HANDLE *pudev, uint8_t *pbuf, uint16_t Len);
uint8_t  USB_CtlTransmitStatus (USB_CORE_HANDLE *pudev);
uint8_t  USB_CtlReceiveStatus  (USB_CORE_HANDLE *pudev);

uint16_t  USB_GetRxCount       (USB_CORE_HANDLE *pudev, uint8_t EpID);

/**
  * @}
  */

#endif /* __GD32F1X0_USB_CORE_H */

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
