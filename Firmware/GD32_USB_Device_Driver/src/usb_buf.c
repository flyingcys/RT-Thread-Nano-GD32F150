/**
  ******************************************************************************
  * @file    usb_buf.c
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   USB buffer interface functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_buf.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Driver
  * @{
  */

/** @defgroup USB_Buffer_Private_functions
  * @{
  */

/**
  * @brief  Free buffer used from application by toggling the SW_BUF byte
  * @param  EpID: endpoint identifier which is in (0..7)
  * @param  Dir: endpoint direction which can be OUT(0) or IN(1)
  * @retval None
  */
void  FreeUserBuffer (uint8_t EpID, uint8_t Dir)
{
    if (Dir == DBUF_EP_OUT)
    {
        _ToggleSWBUF_RX(EpID);
    }
    else if (Dir == DBUF_EP_IN)
    {
        _ToggleSWBUF_TX(EpID);
    }
}

/**
  * @brief  Copy a buffer from user memory area to the allocation buffer area
  * @param  UsrBuf: pointer to user memory area
  * @param  BufAddr: the allocation buffer address
  * @param  Bytes: byte count
  * @retval None
  */
void  UserCopyToBuffer (uint8_t *UsrBuf, uint16_t BufAddr, uint16_t Bytes)
{
    uint32_t n = (Bytes + 1) >> 1;
    uint16_t i, temp;
    uint16_t *BaseAddr = (uint16_t *)(BufAddr * 2 + PBA_ADDR);

    for (i = n; i != 0; i--)
    {
        temp = (uint16_t) *UsrBuf;
        UsrBuf++;
        *BaseAddr = temp | (((uint16_t)*UsrBuf) << 8);
        BaseAddr += 2;
        UsrBuf++;
    }
}

/**
  * @brief  Copy a buffer from the allocation buffer area to user memory area
  * @param  UsrBuf: pointer to user memory area
  * @param  BufAddr: the allocation buffer address
  * @param  Bytes: byte count
  * @retval None
  */
void  BufferCopyToUser (uint8_t *UsrBuf, uint16_t BufAddr, uint16_t Bytes)
{
    uint32_t n = (Bytes + 1) >> 1;
    uint32_t i;
    uint32_t *BaseAddr = (uint32_t *)(BufAddr * 2 + PBA_ADDR);

    for (i = n; i != 0; i--)
    {
        *(uint16_t*)UsrBuf++ = *BaseAddr++;
        UsrBuf++;
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
