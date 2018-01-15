/**
  ******************************************************************************
  * @file    usb_it.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB device interrupt operation functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_it.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/** @defgroup USBD_Interrupt_Handle_Functions
  * @{
  */
static uint8_t  USBINT_LPST    (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_HPST    (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_PMOU    (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_Error   (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_SOF     (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_ESOF    (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_Reset   (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_Suspend (USB_DEVICE_HANDLE *pudev);
static uint8_t  USBINT_WakeUp  (USB_DEVICE_HANDLE *pudev);

/**
  * @}
  */

/** @defgroup USBD_Interrupt_Private_Variables
  * @{
  */
extern __IO uint8_t SuspendEnabled;

USB_INTHandler_TypeDef USB_INTHandler =
{
    USBINT_HPST,
    USBINT_LPST,
    USBINT_PMOU,
    USBINT_Error,
    USBINT_Reset,
    USBINT_SOF,
    USBINT_ESOF,
    USBINT_Suspend,
    USBINT_WakeUp,
};

USB_INTHandler_TypeDef *USB_INT_fops = &USB_INTHandler;

/**
  * @}
  */

/**
  * @brief USBD_Interrupt_Private_Functions
  */

/**
  * @brief  Handle usb low priority successful transfer event 
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_LPST (USB_DEVICE_HANDLE *pudev)
{
    USB_EP *Ep = NULL;
    uint8_t EpID = 0;
    __IO uint16_t Ifr = 0;
    __IO uint16_t EpValue = 0;

    /* Wait till interrupts are not pending */
    while(((Ifr = _GetIFR()) & IFR_STIF) != 0)
    {
        /* Get endpoint number and the value of control and state register */
        EpID = (uint8_t)(Ifr & IFR_EPNUM);
        EpValue = _GetEPxCSR(EpID);

        if((Ifr & IFR_DIR) == 0)
        {
            /* Handle the In direction transaction */

            Ep = &(pudev->dev.in_ep[EpID]);

            if((EpValue & EPTX_ST) != 0)
            {
                /* Clear successful transmit interrupt flag */
                _ClearEPTX_ST(EpID);

                /* Just handle single buffer situation */
                Ep->xfer_count = _GetEPTxCount(EpID);

                /* Maybe mutiple packets */
                Ep->xfer_buf += Ep->xfer_count;

                USBD_DataInStage(pudev, EpID);
            }
        }
        else
        {
            /* Handle the OUT direction transaction */

            int count = 0;

            Ep = &(pudev->dev.out_ep[EpID]);

            if((EpValue & EPRX_ST) != 0)
            {
                /* Clear successful receive interrupt flag */
                _ClearEPRX_ST(EpID);

                count = _GetEPRxCount(EpID);

                if(count != 0)
                {
                    if((EpValue & EP_SETUP) != 0)
                    {
                        /* Handle setup packet */
                        BufferCopyToUser(&(pudev->dev.setup_packet[0]), Ep->bufaddress, count);

                        USBD_SetupStage(pudev);

                        return USBD_OK;
                    }
                    else
                    {
                        BufferCopyToUser(Ep->xfer_buf, Ep->bufaddress, count);
                    }
                }

                /* Maybe mutiple packets */
                Ep->xfer_count += count;
                Ep->xfer_buf += count;
                Ep->xfer_len -= count;

                if(Ep->xfer_len == 0 || count < Ep->maxpacket)
                {
                    USBD_DataOutStage(pudev, EpID);
                    Ep->xfer_count = 0;
                }
                else
                {
                    USB_EP_Rx(pudev, EpID, Ep->xfer_buf, Ep->xfer_len);
                }
            }
        }/* if((temp_Ifr & IFR_DIR) == 0) else ... */    
    }/* while(...) */

    return USBD_OK;
}

/**
  * @brief  Handle usb high priority successful transfer event
  * @param  pudev: pointer to usb device instance 
  * @retval usb device operation status
  */
static uint8_t  USBINT_HPST (USB_DEVICE_HANDLE *pudev)
{
    USB_EP  *Ep = NULL;
    uint8_t  EpID = 0;
    __IO uint16_t Ifr = 0;  
    __IO uint16_t EpValue = 0;

    /* wait till interrupts are not pending */
    while (((Ifr = _GetIFR()) & IFR_STIF) != 0)
    {
        /* Get endpoint number and the value of control and states register */
        EpID = (uint8_t)(Ifr & IFR_EPNUM);
        EpValue = _GetEPxCSR(EpID);

        if((Ifr & IFR_DIR) == 0)
        {
            /* handle the In direction transaction */

            Ep = &(pudev->dev.in_ep[EpID]);

            if((EpValue & EPTX_ST) != 0)
            {
                uint16_t len = 0;
                uint16_t bufaddr = 0;

                _ClearEPTX_ST(EpID);

                if (EpValue & EPTX_DTG)
                {
                    Ep->xfer_count = _GetEPDblBuf0Count(EpID);
                    bufaddr = Ep->buf1addr;
                }
                else
                {
                    Ep->xfer_count = _GetEPDblBuf1Count(EpID);
                    bufaddr = Ep->buf0addr;
                }

                Ep->xfer_buf += Ep->xfer_count;
                Ep->xfer_len -= Ep->xfer_count;

                if(Ep->xfer_len > Ep->maxpacket)
                {
                    len = Ep->maxpacket;
                }
                else if((0 < Ep->xfer_len) && (Ep->xfer_len <= Ep->maxpacket))
                {
                    len = Ep->xfer_len;
                }

                if(len > 0)
                {
                    UserCopyToBuffer(Ep->xfer_buf, bufaddr, len);
                    FreeUserBuffer(EpID, DBUF_EP_IN);
                }

                if(Ep->xfer_len == 0)
                {
                    _SetEPTxStatus(EpID, EPTX_NAK);
                    pudev->dev.class_cb->DataIn(&USB_Device_dev, EpID);
                }
            }
        }
        else
        {
            /* Handle the Out direction transaction */

            Ep = &((&USB_Device_dev)->dev.out_ep[EpID]);

            if((EpValue & EPRX_ST) != 0)
            {
                int count = 0;

                _ClearEPRX_ST(EpID);

                if (EpValue & EPTX_DTG)
                {
                    count = _GetEPDblBuf0Count(EpID);
                    if (count != 0)
                    {
                        BufferCopyToUser(Ep->xfer_buf, Ep->buf0addr, count);
                    }
                }
                else
                {
                    count = _GetEPDblBuf1Count(EpID);
                    if (count != 0)
                    {
                        BufferCopyToUser(Ep->xfer_buf, Ep->buf1addr, count);
                    }
                }

                FreeUserBuffer(EpID, DBUF_EP_OUT);

                /* Maybe multiple packets */
                Ep->xfer_buf += count;
                Ep->xfer_count += count;
                Ep->xfer_len -= count;

                if((Ep->xfer_len == 0) || (count < Ep->maxpacket))
                {
                    _SetEPRxStatus(EpID, EPRX_NAK);
                    pudev->dev.class_cb->DataOut(&USB_Device_dev, EpID);
                    Ep->xfer_count = 0;
                }
            }
        }/* if((temp_Ifr & IFR_DIR) == 0) else ... */    
    }/* while(...) */

    return USBD_OK;
}

/**
  * @brief  Handle usb packet buffer overrun/underrun event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_PMOU (USB_CORE_HANDLE *pudev)
{
    return USBD_OK;
}

/**
  * @brief  Handle error event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_Error (USB_CORE_HANDLE *pudev)
{
    return USBD_OK;
}

/**
  * @brief  Handle usb SOF event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_SOF (USB_CORE_HANDLE *pudev)
{
    if(pudev->dev.class_cb->SOF)
    {
        pudev->dev.class_cb->SOF(pudev); 
    }
    return USBD_OK;
}

/**
  * @brief  Handle usb expect SOF event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_ESOF (USB_CORE_HANDLE *pudev)
{
    /* Control resume time by ESOFs */
    USBD_Resume(RESUME_ESOF); 

    return USBD_OK;
}

/**
  * @brief  Handle usb reset event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_Reset (USB_CORE_HANDLE *pudev)
{
#ifdef LPM_ENABLED
    uint16_t sLPM;
#endif

    USB_EP_BufConfig(pudev, 0x00, USB_SNG_BUFTYPE, EP0_RX_ADDRESS);
    USB_EP_BufConfig(pudev, 0x80, USB_SNG_BUFTYPE, EP0_TX_ADDRESS);

    /* Initialize endpoint0 in OUT direction */
    USB_EP_Init(pudev,
                EP0_OUT,
                USB_EPTYPE_CONTROL,
                USB_EP0_MAX_SIZE);

    /* Initialize endpoint0 in IN direction */
    USB_EP_Init(pudev,
                EP0_IN,
                USB_EPTYPE_CONTROL,
                USB_EP0_MAX_SIZE);

    /* Set device address as default address 0 */
    USB_EP_SetAddress(pudev, 0);

#ifdef LPM_ENABLED
    sLPM = SUBEP_VALID;
    _SetLPM_CNTR(0x8000);
    _SetSUBEP0R(sLPM);
#endif

    /* Endpoint 0 prepare to receive setup packet */
    USB_EP_Rx(pudev, EP0_OUT, pudev->dev.setup_packet, 8);

    pudev->dev.device_cur_status = USB_STATUS_DEFAULT;

#ifdef USE_LCD_LOG
    /* Call back user reset function */
    pudev->dev.user_cb->DeviceReset(pudev->dev.speed);
#endif

    return USBD_OK;
}

/**
  * @brief  Handle usb suspend event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_Suspend (USB_CORE_HANDLE *pudev)
{
    /* Store the device current status */
    pudev->dev.device_old_status = pudev->dev.device_cur_status;

    /* Set device in suspended state */
    pudev->dev.device_cur_status  = USB_STATUS_SUSPENDED;

#ifdef USE_LCD_LOG
    /* Call back user suspend function */
    pudev->dev.user_cb->DeviceSuspended();
#endif

    /* Enter usb in suspend and mcu system in low power mode */
    if (SuspendEnabled)
    {
        USBD_Suspend();
    }
    else
    {
        /* if not possible then resume after xx ms */
        USBD_Resume(RESUME_LATER);
    }

    return USBD_OK;
}

/**
  * @brief  Handle usb wakeup event
  * @param  pudev: pointer to usb device instance
  * @retval usb device operation status
  */
static uint8_t  USBINT_WakeUp (USB_CORE_HANDLE *pudev)
{
#ifdef USE_LCD_LOG
    /* Call back user resume function */
    pudev->dev.user_cb->DeviceResumed();
#endif

    /* Restore the old status */
    pudev->dev.device_cur_status = pudev->dev.device_old_status;

    USBD_Resume(RESUME_EXTERNAL);

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
