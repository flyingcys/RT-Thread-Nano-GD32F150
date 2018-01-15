/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device core interface functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

/* Private variables ---------------------------------------------------------*/
extern uint8_t gAddress;

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/** @defgroup USBD_Core_Private_Functions
  * @{
  */

/**
  * @brief  Configure usb device to be connected
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  USBD_Connect (USB_DEVICE_HANDLE *pudev)
{
    USB_Cable_Config(ENABLE);
    pudev->dev.device_cur_status = USB_STATUS_CONNECTED;
}

/**
  * @brief  Configure usb device to be unconnected
  * @param  pudev: pointer to usb device instance
  * @retval None
  */
void  USBD_Disconnect (USB_DEVICE_HANDLE *pudev)
{
    USB_Cable_Config(DISABLE);
    DR_StopDevice();
    pudev->dev.device_cur_status = USB_STATUS_UNCONNECTED;
}

/**
  * @brief  Initialize usb device stack
  * @param  pudev: pointer to usb device instance
  * @param  device_desc: device descriptor callback structure pointer
  * @param  class_cb: class callback structure pointer
  * @param  user_cb: user callback structure pointer
  * @retval None
  */
void  USBD_Init (USB_DEVICE_HANDLE *pudev,
                 USBD_Desc_cb_TypeDef  *device_desc,
                 USBD_Class_cb_TypeDef *class_cb,
                 USBD_User_cb_TypeDef  *user_cb)
{
    USBD_DeInit(pudev);

    /* Device descriptor class and user callbacks */
    pudev->dev.class_cb = class_cb;
    pudev->dev.user_cb = user_cb;
    pudev->dev.desc_cb = device_desc;

    /* Get serial string from the unique ID */
    Get_SerialString();

    /* Disable remote wakeup feature */
    pudev->dev.DevRemoteWakeup = 0;

    /* USB full-speed device */
    pudev->dev.speed = USB_SPEED_FULL;

    /* Set device register */
    DR_Init();

#ifdef USE_LCD_LOG
    /* Callback user function */
    pudev->dev.user_cb->DeviceInit();
#endif
}

/**
  * @brief  De-Initialize USB device stack
  * @param  pudev: pointer to USB device instance
  * @retval usb device operation status
  */
USBD_Status  USBD_DeInit (USB_DEVICE_HANDLE *pudev)
{
    return USBD_OK;
}

/**
  * @brief  Usb setup stage processing
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
uint8_t  USBD_SetupStage (USB_DEVICE_HANDLE *pudev)
{
    USB_DEVICE_REQ req;

    USBD_ParseSetupRequest(pudev, &req);

    switch (req.bmRequestType & USB_REQ_MASK) 
    {
        /* Standard device request */
        case USB_STANDARD_REQ:
            USBD_StdReq(pudev, &req);
            break;

        /* Device class request */
        case USB_CLASS_REQ:
            USBD_DevClsReq(pudev, &req);
            break;

        /* Vendor defined request */
        case USB_VENDOR_REQ:
            USBD_VdrDefReq(pudev, &req);
            break;

        default:
            USB_EP_Stall(pudev, 0x00);
            break;
    }
    return USBD_OK;
}

/**
  * @brief  Data out stage processing
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
uint8_t  USBD_DataOutStage (USB_DEVICE_HANDLE *pudev, uint8_t EpID)
{
    USB_EP *ep = &pudev->dev.out_ep[EpID];

    if(EpID == 0) 
    {
        if(pudev->dev.device_cur_state == USB_CTRL_DATA_OUT)
        {
            if(ep->xfer_len > ep->maxpacket)
            {
                /* One data packet has been received, update xfer_len */
                ep->xfer_len -= ep->maxpacket;

                /* Continue to receive remain data */
                USB_CtlContinueRx(pudev, ep->xfer_buf, ep->xfer_len);
            }
            else
            {
                if((pudev->dev.class_cb->EP0_RxReady != NULL)&&
                    (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED))
                {
                    /* Device class handle */
                    pudev->dev.class_cb->EP0_RxReady(pudev);
                }

                /* Enter the control transaction status stage */
                USB_CtlTransmitStatus(pudev);
            }
        }
        else if( pudev->dev.device_cur_state == USB_CTRL_STATUS_OUT)
        {
            /* Clear endpoint status_out status */
            _Clear_Status_Out(EP0);
        }
    }
    else 
    {
        if((pudev->dev.class_cb->DataOut != NULL)&&
            (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED))
        {
            pudev->dev.class_cb->DataOut(pudev, EpID);
        }
    }
    return USBD_OK;
}

/**
  * @brief  Data in stage processing
  * @param  pudev: pointer to usb device instance
  * @param  EpID: endpoint identifier which is in (0..7)
  * @retval usb device operation status
  */
uint8_t  USBD_DataInStage (USB_DEVICE_HANDLE *pudev, uint8_t EpID)
{
    USB_EP *ep = &pudev->dev.in_ep[EpID];

    if(EpID == 0) 
    {
        if(pudev->dev.device_cur_state == USB_CTRL_DATA_IN)
        {
            if(ep->xfer_len > ep->maxpacket)
            {
                /* One data packet has been transmited, update xfer_len */
                ep->xfer_len -= ep->maxpacket;

                /* Continue to receive remain data */
                USB_CtlContinueTx(pudev, ep->xfer_buf, ep->xfer_len);
            }
            else
            {
#ifndef USB_DFU
                /* Transmit length is maxpacket multiple, so send zero length packet */
                if(((ep->xfer_len % ep->maxpacket) == 0) && (ep->xfer_len < ep->ctrl_count))
                {
                    USB_CtlContinueTx(pudev, NULL, 0);

                    ep->ctrl_count = 0;
                }
                else
#endif /* USB_DFU */
                {
                    ep->xfer_len = 0;

                    if((pudev->dev.class_cb->EP0_TxSent != NULL) &&
                        (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED))
                    {
                        pudev->dev.class_cb->EP0_TxSent(pudev); 
                    }

                    USB_CtlReceiveStatus(pudev);
                }
            }
        }
        else if((pudev->dev.device_cur_state == USB_CTRL_STATUS_IN) && (gAddress != 0))
        {
            USB_EP_SetAddress(pudev, gAddress); 
            gAddress = 0;
        }
    }
    else
    {
        ep->xfer_len -= ep->xfer_count;

        if(ep->xfer_len == 0)
        {
            if((pudev->dev.class_cb->DataIn != NULL) &&
                (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED))
            {
                pudev->dev.class_cb->DataIn(pudev, EpID);
            }
        }
        else
        {
            USB_EP_Tx(pudev, EpID, ep->xfer_buf, ep->xfer_len);
        }
    }
    return USBD_OK;
}

/**
  * @brief  Device configuration and interface setting
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */

USBD_Status  USBD_SetCfg (USB_DEVICE_HANDLE *pudev, uint8_t ConfigIndex)
{
    /* Configure USB device class */
    pudev->dev.class_cb->Init(pudev, ConfigIndex);

#ifdef USE_LCD_LOG
    pudev->dev.user_cb->DeviceConfigured();
#endif

    return USBD_OK; 
}

/**
  * @brief  Clear current configuration
  * @param  pudev: pointer to usb device instance
  * @param  ConfigIndex: configuration index
  * @retval usb device operation status
  */
USBD_Status  USBD_ClrCfg (USB_DEVICE_HANDLE *pudev, uint8_t ConfigIndex)
{
    pudev->dev.class_cb->DeInit(pudev, ConfigIndex);

    return USBD_OK;
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

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/

