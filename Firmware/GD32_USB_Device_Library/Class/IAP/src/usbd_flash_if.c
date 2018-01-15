/**
  ******************************************************************************
  * @file    usbd_flash_if.c
  * @author  MCU SD
  * @version V1.0
  * @date    6-Sep-2014
  * @brief   USB DFU device flash interface functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_flash_if.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup USB
  * @{
  */

/** @addtogroup USB_Device_Class_Library
  * @{
  */

/** @addtogroup USB_DFU_Class
  * @{
  */

/** @defgroup USB_DFU_FlashIf_Private_Functions
  * @{
  */
static uint8_t  Flash_If_Init      (void);
static uint8_t  Flash_If_DeInit    (void);
static uint8_t  Flash_If_Erase     (uint32_t Addr, uint32_t Page_Count);
static uint8_t  Flash_If_Write     (uint8_t Data[], uint32_t Addr, uint32_t Len);
static uint8_t* Flash_If_Read      (uint32_t Addr, uint32_t Len);
static uint8_t  Flash_If_CheckAddr (uint32_t Addr);

/**
  * @}
  */

/** @defgroup USB_DFU_FlashIf_Private_Variables
  * @{
  */
IAP_MAL_Property_TypeDef IAP_Flash_cb =
{
    FLASH_IF_STRING,
    Flash_If_Init,
    Flash_If_DeInit,
    Flash_If_Erase,
    Flash_If_Write,
    Flash_If_Read,
    Flash_If_CheckAddr,
    60, /* Flash erase timeout in ms */
    80  /* Flash programming timeout in ms (80us * RAM Buffer size (1024 Bytes) */
};

/** @defgroup USB_DFU_FlashIf_Private_Functions
  * @{
  */

/**
  * @brief  Flash memory interface initialization routine.
  * @param  None
  * @retval MAL_OK if the operation is right, MAL_FAIL else
*/
static uint8_t  Flash_If_Init (void)
{
    /* Unlock the internal flash */
    FMC_Unlock();

    return MAL_OK;
}

/**
  * @brief  Flash memory interface deinitialization routine.
  * @param  None
  * @retval MAL_OK if all operation is right, MAL_FAIL else
  */
static uint8_t  Flash_If_DeInit (void)
{
    /* Lock the internal flash */
    FMC_Lock();

    return MAL_OK;
}

/**
  * @brief  Erase flash sector
  * @param  Addr: address to be written to.
  * @retval MAL_OK
  */
static uint8_t  Flash_If_Erase (uint32_t Addr, uint32_t PageCount)
{
    int i;

    for(i = 0; i < PageCount; i ++)
    {
        /* Call the standard flash erase-page function */
        FMC_ErasePage(Addr);

        Addr += PAGE_SIZE;
    }

    return MAL_OK;
}

/**
  * @brief  Flash memory write routine
  * @param  Addr: address to be written to
  * @param  Len: length of data to be written (in bytes).
  * @retval MAL_OK
  */
static uint8_t  Flash_If_Write (uint8_t Data[], uint32_t Addr, uint32_t Len)
{
    uint32_t idx = 0;

    FMC_Unlock();

    if (Len & 0x03) /* Not an aligned data */
    {
        for (idx = Len; idx < ((Len & 0xFFFC) + 4); idx++)
        {
            Data[idx] = 0xFF;
        }
    }

    /* Data received are word multiple */
    for (idx = 0; idx < Len; idx += 4)
    {
        FMC_ProgramWord(Addr, *(uint32_t *)(Data + idx));
        Addr += 4;
    }

    return MAL_OK;
}

/**
  * @brief  Flash memory read routine
  * @param  Addr: address to be read from
  * @param  Len: length of data to be read (in bytes)
  * @retval Pointer to the physical address where data should be read
  */
static uint8_t*  Flash_If_Read (uint32_t Addr, uint32_t Len)
{
    return (uint8_t *)(Addr);
}

/**
  * @brief  Check if the address is an allowed address for this memory
  * @param  Addr: address to be checked.
  * @retval MAL_OK if the address is allowed, MAL_FAIL else.
  */
static uint8_t  Flash_If_CheckAddr (uint32_t Addr)
{
    if ((Addr >= FLASH_START_ADDR) && (Addr < FLASH_END_ADDR))
    {
        return MAL_OK;
    }
    else
    {
        return MAL_FAIL;
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

/**
  * @}
  */

/************************ (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
