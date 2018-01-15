/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCU SD 
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   This file provides the functions needed by USB enumeration
  * @note    About USB enumeration, please refer to the USB2.0 protocol
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_enum.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Core_Library
  * @{
  */

/* USB enumeration handle functions */
static void USBD_GetDescriptor     (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SetAddress        (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SetConfiguration  (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_GetConfiguration  (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_GetStatus         (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SetFeature        (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_ClearFeature      (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_Reserved          (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SetDescriptor     (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_GetInterface      (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SetInterface      (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);
static void USBD_SynchFrame        (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req);

/** @defgroup USBD_Enumeration_Private_Variables
  * @{
  */
uint8_t gAddress = 0;
uint8_t USBD_StrDesc[USB_STR_DESC_MAX_SIZE];

static void (*StandardDeviceRequest[])(USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req) =
{
    USBD_GetStatus,
    USBD_ClearFeature,
    USBD_Reserved,
    USBD_SetFeature,
    USBD_Reserved,
    USBD_SetAddress,
    USBD_GetDescriptor,
    USBD_SetDescriptor,
    USBD_GetConfiguration,
    USBD_SetConfiguration,
    USBD_GetInterface,
    USBD_SetInterface,
    USBD_SynchFrame,
};

/**
  * @}
  */

/** @defgroup USBD_Enumeration_Private_Functions
  * @{
  */

/**
  * @brief  Handle usb standard device request
  * @param  pudev: pointer to usb device instance
  * @param  req: USB device request
  * @retval usb device operation status
  */
uint8_t  USBD_StdReq (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    /* Call device request handle function */
    (*StandardDeviceRequest[req->bRequest])(pudev, req);

    return USBD_OK;
}

/**
  * @brief  Handle device class request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device class request
  * @retval usb device operation status
  */
uint8_t  USBD_DevClsReq (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    USBD_Status ret;

    switch (pudev->dev.device_cur_status) 
    {
        case USB_STATUS_CONFIGURED:
            if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM) 
            {
                /* Call back device class handle function */
                ret = (USBD_Status)(pudev->dev.class_cb->ClassReqHandle(pudev, req));

                if((req->wLength == 0) && (ret == USBD_OK))
                {
                    /* No data stage */
                    USB_CtlTransmitStatus(pudev);
                }
            }
            else
            {
                USBD_EnumError(pudev, req);
            }
            break;

        default:
            USBD_EnumError(pudev, req);
            break;
    }

    return USBD_OK;
}

/**
  * @brief  Handle usb vendor request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb vendor request
  * @retval usb device operation status
  */
uint8_t  USBD_VdrDefReq (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    /* Added by user... */

    return USBD_OK;
}

/**
  * @brief  No operation, just for reserved
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_Reserved (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    /* No operation... */
}

/**
  * @brief  Handle usb Get_Status request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_GetStatus (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    uint8_t  ep_addr;
    uint16_t config_status = 0x0000;
    uint16_t endp_status = 0x0000;

    switch(req->bmRequestType & USB_REQ_RECIPIENT_MASK)
    {
        case USB_REQTYPE_DEVICE:
            switch (pudev->dev.device_cur_status) 
            {
                case USB_STATUS_ADDRESSED:
                case USB_STATUS_CONFIGURED:

#ifdef USBD_SELF_POWERED
    config_status = USB_STATUS_SELF_POWERED;
#endif

                    if (pudev->dev.DevRemoteWakeup) 
                    {
                        config_status |= USB_STATUS_REMOTE_WAKEUP;
                    }
                    USB_CtlTx(pudev, (uint8_t *)&config_status, 2);
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_INTERFACE:
            switch (pudev->dev.device_cur_status)
            {
                case USB_STATUS_ADDRESSED:
                    USBD_EnumError(pudev, req);
                    break;

                case USB_STATUS_CONFIGURED:
                    if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
                    {
                        USB_CtlTx(pudev, (uint8_t *)&config_status, 2);
                    }
                    else
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_ENDPOINT:
            ep_addr = LOWBYTE(req->wIndex);

            switch (pudev->dev.device_cur_status) 
            {
                case USB_STATUS_ADDRESSED:
                    if (IS_NOT_EP0(ep_addr)) 
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                case USB_STATUS_CONFIGURED:
                    if ((ep_addr & 0x80)== 0x80)
                    {
                        if(pudev->dev.in_ep[ep_addr & 0x7F].is_stall)
                        {
                            endp_status = 0x0001;
                        }
                    }
                    else
                    {
                        if(pudev->dev.out_ep[ep_addr].is_stall)
                        {
                            endp_status = 0x0001;
                        }
                    }
                    USB_CtlTx(pudev, (uint8_t *)&endp_status, 2);
                    break;

                default:
                    break;
            }
            break;

        default:
            USBD_EnumError(pudev, req);
            break;
    }
}

/**
  * @brief  Handle usb Clear_Feature request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_ClearFeature (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    uint8_t ep_addr = 0;

    switch(req->bmRequestType & USB_REQ_RECIPIENT_MASK)
    {
        case USB_REQTYPE_DEVICE:
            switch (pudev->dev.device_cur_status)
            {
                case USB_STATUS_ADDRESSED:
                case USB_STATUS_CONFIGURED:
                    if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
                    {
                        pudev->dev.DevRemoteWakeup = 0;
                        pudev->dev.class_cb->ClearFeature(pudev, req);
                        USB_CtlTransmitStatus(pudev);
                    }
                    else if(req->wValue == USB_FEATURE_TEST_MODE)
                    {
                        /* Can not clear test_mode feature */
                        USBD_EnumError(pudev, req);
                    }
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_INTERFACE:
            switch(pudev->dev.device_cur_state)
            {
                case USB_STATUS_ADDRESSED:
                    USBD_EnumError(pudev, req);
                    break;

                case USB_STATUS_CONFIGURED:
                    if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
                    {/* No operation */
                    }
                    else
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_ENDPOINT:
            ep_addr = LOWBYTE(req->wIndex);

            switch (pudev->dev.device_cur_status) 
            {
                case USB_STATUS_ADDRESSED:
                    if (IS_NOT_EP0(ep_addr))
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                case USB_STATUS_CONFIGURED:   
                    if (req->wValue == USB_FEATURE_ENDP_HALT)
                    {
                        if (IS_NOT_EP0(ep_addr))
                        {
                            USB_EP_ClrStall(pudev, ep_addr);
                        }
                    }
                    pudev->dev.class_cb->ClearFeature(pudev, req);
                    USB_CtlTransmitStatus(pudev);
                    break;

                default:
                    break;
            }
            break;

        default:
            USBD_EnumError(pudev, req);
            break;
    }
}

/**
  * @brief  Handle usb Set_Feature request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SetFeature (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    uint8_t ep_addr = 0;

    switch(req->bmRequestType & USB_REQ_RECIPIENT_MASK)
    {
        case USB_REQTYPE_DEVICE:
            switch (pudev->dev.device_cur_status)
            {
                case USB_STATUS_ADDRESSED:
                case USB_STATUS_CONFIGURED:
                    if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
                    {
                        pudev->dev.DevRemoteWakeup = 1;
                        USB_CtlTransmitStatus(pudev);
                    }
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_INTERFACE:
            switch(pudev->dev.device_cur_state)
            {
                case USB_STATUS_ADDRESSED:
                    USBD_EnumError(pudev, req);
                    break;

                case USB_STATUS_CONFIGURED:
                    if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
                    {/* No operation */
                    }
                    else
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                default:
                    break;
            }
            break;

        case USB_REQTYPE_ENDPOINT:
            switch (pudev->dev.device_cur_status) 
            {
                case USB_STATUS_ADDRESSED:
                    if (IS_NOT_EP0(ep_addr))
                    {
                        USBD_EnumError(pudev, req);
                    }
                    break;

                case USB_STATUS_CONFIGURED:   
                    if (req->wValue == USB_FEATURE_ENDP_HALT)
                    {
                        if (IS_NOT_EP0(ep_addr)) 
                        {
                            USB_EP_Stall(pudev, ep_addr);
                        }
                    }
                    USB_CtlTransmitStatus(pudev);
                    break;

                default:
                    break;
            }
            break;

        default:
            USBD_EnumError(pudev, req);
            break;
    }
}

/**
  * @brief  Handle usb Set_Address request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SetAddress (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    if ((req->wIndex == 0) && (req->wLength == 0))
    {
        gAddress = (uint8_t)(req->wValue) & 0x7F;

        if (pudev->dev.device_cur_status == USB_STATUS_CONFIGURED) 
        {
            USBD_EnumError(pudev, req);
        }
        else
        {
            USB_CtlTransmitStatus(pudev);

            if (gAddress != 0) 
            {
                pudev->dev.device_cur_status  = USB_STATUS_ADDRESSED;
            }
            else
            {
                pudev->dev.device_cur_status  = USB_STATUS_DEFAULT;
            }
        }
    }
    else 
    {
        USBD_EnumError(pudev , req);
    }
}

/**
  * @brief  Handle usb Get_Descriptor request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_GetDescriptor (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    if((req->bmRequestType & USB_REQ_RECIPIENT_MASK) == USB_REQTYPE_DEVICE)
    {
        uint16_t len;
        uint8_t *pbuf;

        switch (req->wValue >> 8)
        {
#ifdef LPM_ENABLED    
            case USB_DESC_TYPE_BOS:
                pbuf = pudev->dev.desc_cb->GetBOSDescriptor(pudev->dev.speed, &len);
                break;
#endif
            case USB_DESCTYPE_DEVICE:
                pbuf = pudev->dev.desc_cb->GetDeviceDescriptor(pudev->dev.speed, &len);
                if (req->wLength == 64) len = 8;
                break;

            case USB_DESCTYPE_CONFIGURATION:
                pbuf = (uint8_t *)pudev->dev.class_cb->GetConfigDescriptor(pudev->dev.speed, &len);
                break;

            case USB_DESCTYPE_STRING:
                switch ((uint8_t)(req->wValue))
                {
                    case USBD_LANGID_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetLangIDStrDescriptor(pudev->dev.speed, &len);
                        break;

                    case USBD_MFC_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetManufacturerStrDescriptor(pudev->dev.speed, &len);
                        break;

                    case USBD_PRODUCT_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetProductStrDescriptor(pudev->dev.speed, &len);
                        break;

                    case USBD_SERIAL_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetSerialStrDescriptor(pudev->dev.speed, &len);
                        break;

                    case USBD_CONFIG_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetConfigurationStrDescriptor(pudev->dev.speed, &len);
                        break;

                    case USBD_INTERFACE_STR_IDX:
                        pbuf = pudev->dev.desc_cb->GetInterfaceStrDescriptor(pudev->dev.speed, &len);
                        break;

                    default:  
#ifdef USB_SUPPORT_USER_STRING_DESC
                        pbuf = pudev->dev.class_cb->GetUsrStrDescriptor(pudev->dev.speed, (req->wValue), &len);
                        break;
#else
                        USBD_EnumError(pudev, req);
                        return;
#endif  
                }
                break;

            case USB_DESCTYPE_DEVICE_QUALIFIER:
                USBD_EnumError(pudev, req);
                return;  

            case USB_DESCTYPE_OTHER_SPEED_CONFIGURATION:
                USBD_EnumError(pudev, req);
                return;

            default: 
                USBD_EnumError(pudev, req);
                return;
        }

        if((len != 0) && (req->wLength != 0))
        {
            len = MIN(len, req->wLength);

            USB_CtlTx (pudev, pbuf,len);
        }
    }
    else if((req->bmRequestType & USB_REQ_RECIPIENT_MASK) == USB_REQTYPE_INTERFACE)
    {
        pudev->dev.class_cb->GetClassDescriptor(pudev, req);
    }
}

/**
  * @brief  Handle usb Set_Descriptor request
  * @param  pudev: usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SetDescriptor (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    /* No handle... */
}

/**
  * @brief  Handle usb Get_Configuration request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_GetConfiguration (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    uint32_t USBD_default_config = 0;

    if (req->wLength != 1)
    {
        USBD_EnumError(pudev, req);
    }
    else 
    {
        switch (pudev->dev.device_cur_status)
        {
            case USB_STATUS_ADDRESSED:
                USB_CtlTx (pudev, (uint8_t *)&USBD_default_config, 1);
                break;

            case USB_STATUS_CONFIGURED:
                USB_CtlTx (pudev, &pudev->dev.device_cur_config, 1);
                break;

            default:
                break;
        }
    }
}

/**
  * @brief  Handle usb Set_Configuration request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SetConfiguration (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    static uint8_t  cfgidx;

    cfgidx = (uint8_t)(req->wValue);

    if (cfgidx > USBD_CFG_MAX_NUM) 
    {
        USBD_EnumError(pudev, req);
    }
    else 
    {
        switch (pudev->dev.device_cur_status) 
        {
            case USB_STATUS_ADDRESSED:
                if (cfgidx)
                {
                    pudev->dev.device_cur_config = cfgidx;
                    pudev->dev.device_cur_status = USB_STATUS_CONFIGURED;
                    USBD_SetCfg(pudev, cfgidx);
                    USB_CtlTransmitStatus(pudev);
                }
                else
                {
                    USB_CtlTransmitStatus(pudev);
                }
                break;

            case USB_STATUS_CONFIGURED:
                if (cfgidx == 0) 
                {
                    pudev->dev.device_cur_status = USB_STATUS_ADDRESSED;
                    pudev->dev.device_cur_config = cfgidx;
                    USBD_ClrCfg(pudev, cfgidx);
                    USB_CtlTransmitStatus(pudev);
                }
                else if (cfgidx != pudev->dev.device_cur_config)
                {
                    /* Clear old configuration */
                    USBD_ClrCfg(pudev, pudev->dev.device_cur_config);

                    /* Set new configuration */
                    pudev->dev.device_cur_config = cfgidx;
                    USBD_SetCfg(pudev, cfgidx);
                    USB_CtlTransmitStatus(pudev);
                }
                else
                {
                    USB_CtlTransmitStatus(pudev);
                }
                break;

            default:
                break;
        }
    }
}

/**
  * @brief  Handle usb Get_Interface request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_GetInterface (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    switch (pudev->dev.device_cur_status) 
    {
        case USB_STATUS_ADDRESSED:
            USBD_EnumError(pudev, req);
            break;

        case USB_STATUS_CONFIGURED:
            if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
            {
                pudev->dev.class_cb->GetInterface (pudev, req);
            }
            else
            {
                USBD_EnumError(pudev, req);
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  Handle usb Set_Interface request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SetInterface (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    switch (pudev->dev.device_cur_status) 
    {
        case USB_STATUS_ADDRESSED:
            USBD_EnumError(pudev, req);
            break;

        case USB_STATUS_CONFIGURED:
            if (LOWBYTE(req->wIndex) <= USBD_ITF_MAX_NUM)
            {
                pudev->dev.class_cb->SetInterface (pudev, req);
            }
            else
            {
                USBD_EnumError(pudev, req);
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  Handle usb SynchFrame request
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */
static void  USBD_SynchFrame (USB_DEVICE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    /* No handle... */
}


/**
  * @brief  Decode setup data packet
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */

void  USBD_ParseSetupRequest (USB_CORE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    req->bmRequestType = *(uint8_t *)  (pudev->dev.setup_packet);
    req->bRequest      = *(uint8_t *)  (pudev->dev.setup_packet + 1);
    req->wValue        = SWAPBYTE      (pudev->dev.setup_packet + 2);
    req->wIndex        = SWAPBYTE      (pudev->dev.setup_packet + 4);
    req->wLength       = SWAPBYTE      (pudev->dev.setup_packet + 6);

    pudev->dev.in_ep[0].ctrl_count = req->wLength;
    pudev->dev.device_cur_state = USB_CTRL_SETUP;
}

/**
  * @brief  Handle usb enumeration error event
  * @param  pudev: pointer to usb device instance
  * @param  req: usb device request
  * @retval None
  */

void  USBD_EnumError (USB_CORE_HANDLE *pudev, USB_DEVICE_REQ *req)
{
    USB_EP_Stall(pudev, EP0);
}

/**
  * @brief  Convert Hex 32bits value into unicode char
  * @param  Value: Hex 32bits value
  * @param  pbuf: buffer pointer to store unicode char
  * @param  Len: value length
  * @retval None
  */
void IntToUnicode (uint32_t Value, uint8_t *pbuf, uint8_t Len)
{
    uint8_t Index = 0;

    for(Index = 0; Index < Len; Index++)
    {
        if((Value >> 28) < 0x0A)
        {
            pbuf[2 * Index] = (Value >> 28) + '0';
        }
        else
        {
            pbuf[2 * Index] = (Value >> 28) + 'A' - 10; 
        }

        Value = Value << 4;

        pbuf[2 * Index + 1] = 0;
    }
}

/**
  * @brief  Convert normal string into unicode one
  * @param  desc: descriptor string pointer
  * @param  unicode: formatted string (unicode)
  * @param  len: descriptor length pointer
  * @retval None
  */
void  USBD_GetUnicodeString (uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
    uint8_t idx = 1;

    if (desc != NULL) 
    {
        unicode[idx++] =  USB_DESCTYPE_STRING;

        for (; *desc != NULL; ) 
        {
            unicode[idx++] = *desc++;
            unicode[idx++] =  0x00;
        }

        *len = idx;
        unicode[0] = *len;
    }
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
