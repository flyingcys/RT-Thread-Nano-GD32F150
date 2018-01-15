/**
  ******************************************************************************
  * @file    usbd_iap_mal.c
  * @author  MCU SD
  * @version V1.0
  * @date    6-Sep-2014
  * @brief   USB DFU device media access layer functions
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_iap_mal.h"
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

/** @addtogroup USB_IAP
  * @{
  */

/** @defgroup USB_IAP_MAL_Private_Variables
  * @{
  */

/* The reference tables of global memories callback and string descriptors.
   To add a new memory, You can do as follows: 
   1. Modify the value of MAX_USED_MEDIA in usbd_dfu_mal.h
   2. Add the pointer to the callback structure in this table
   3. Add the pointer to the memory string descriptor in usbd_dfu_StringDesc table
   No other operation is required.
 */
IAP_MAL_Property_TypeDef* tMALTab[MAX_USED_MEMORY_MEDIA] = {
    &IAP_Flash_cb
};

/* The list of memory interface string descriptor pointers. This list
   can be updated whenever a memory has to be added or removed */
const uint8_t* USBD_IAP_StringDesc[MAX_USED_MEMORY_MEDIA] = {
    FLASH_IF_STRING
};

/* Memory buffer for downloaded data */
uint8_t  MAL_Buffer[TRANSFER_SIZE];

/**
  * @}
  */

static uint8_t  IAP_MAL_CheckAddr (uint32_t Addr);

/** @defgroup USB_DFU_MAL_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the memory media on the GD32
  * @param  None
  * @retval MAL_OK
  */
uint8_t  IAP_MAL_Init (void)
{
    uint32_t memIdx = 0;

    /* Initialize all supported memory medias */
    for(memIdx = 0; memIdx < MAX_USED_MEMORY_MEDIA; memIdx++)
    {
        /* Check if the memory media exists */
        if (tMALTab[memIdx]->pMAL_Init != NULL)
        {
            tMALTab[memIdx]->pMAL_Init();
        }
    }

    return MAL_OK;
}

/**
  * @brief  Deinitialize the memory media on the GD32
  * @param  None
  * @retval MAL_OK
  */
uint8_t  IAP_MAL_DeInit (void)
{
    uint32_t memIdx = 0;

    /* Deinitializes all supported memory medias */
    for(memIdx = 0; memIdx < MAX_USED_MEMORY_MEDIA; memIdx++)
    {
        /* Check if the memory media exists */
        if (tMALTab[memIdx]->pMAL_DeInit != NULL)
        {
            tMALTab[memIdx]->pMAL_DeInit();
        }
    }

    return MAL_OK;
}

/**
  * @brief  Erase a memory sector
  * @param  Addr: memory sector address/code
  * @retval MAL_OK if all operations are OK, MAL_FAIL else
  */
uint8_t  IAP_MAL_Erase (uint32_t Addr, uint32_t PageCount)
{
    uint32_t memIdx = IAP_MAL_CheckAddr(Addr);

    /* Check if the address is in protected area */
    if (IS_PROTECTED_AREA(Addr))
    {
        return MAL_FAIL;
    }

    if (memIdx < MAX_USED_MEMORY_MEDIA)
    {
        /* Check if the operation is supported */
        if (tMALTab[memIdx]->pMAL_Erase != NULL)
        {
            return tMALTab[memIdx]->pMAL_Erase(Addr, PageCount);
        }
        else
        {
            return MAL_FAIL;
        }
    }
    else
    {
        return MAL_FAIL;
    }
}

/**
  * @brief  Write data to sectors of memory
  * @param  Addr: sector address/code
  * @param  Len: length of data to be written (in bytes)
  * @retval MAL_OK if all operations are OK, MAL_FAIL else
  */
uint8_t  IAP_MAL_Write (uint8_t Data[], uint32_t Addr, uint32_t Len)
{
    uint32_t memIdx = IAP_MAL_CheckAddr(Addr);

    /* Check if the address is in protected area */
    if (IS_PROTECTED_AREA(Addr))
    {
        return MAL_FAIL;
    }

    if (memIdx < MAX_USED_MEMORY_MEDIA)
    {
        /* Check if the operation is supported */
        if (tMALTab[memIdx]->pMAL_Write != NULL)
        {
            return tMALTab[memIdx]->pMAL_Write(Data, Addr, Len);
        }
        else
        {
            return MAL_FAIL;
        }
    }
    else
    {
        return MAL_FAIL;
    }
}

/**
  * @brief  Read data from sectors of memory
  * @param  Addr: sector address/code
  * @param  Len: length of data to be written (in bytes)
  * @retval Pointer to buffer
  */
uint8_t*  IAP_MAL_Read (uint32_t Addr, uint32_t Len)
{
    uint32_t memIdx = 0;

    if(Addr != OB_RDPT)
    {
        memIdx = IAP_MAL_CheckAddr(Addr);
    }

    if (memIdx < MAX_USED_MEMORY_MEDIA)
    {
        /* Check if the operation is supported */
        if (tMALTab[memIdx]->pMAL_Read != NULL)
        {
            return tMALTab[memIdx]->pMAL_Read(Addr, Len);
        }
        else
        {
            return MAL_Buffer;
        }
    }
    else
    {
        return MAL_Buffer;
    }
}

/**
  * @brief  Get the status of a given memory and store in buffer
  * @param  Addr: sector address/code (allow to determine which memory will be addressed)
  * @param  Cmd: 0 for erase and 1 for write
  * @param  buffer: pointer to the buffer where the status data will be stored
  * @retval MAL_OK if all operations are OK, MAL_FAIL else
  */
uint8_t  IAP_MAL_GetStatus (uint32_t Addr, uint8_t Cmd, uint8_t *buffer)
{
    uint32_t memIdx = IAP_MAL_CheckAddr(Addr);

    if (memIdx < MAX_USED_MEMORY_MEDIA)
    {
        if (Cmd & 0x01)
        {
            SET_POLLING_TIMEOUT(tMALTab[memIdx]->WriteTimeout);
        }
        else
        {
            SET_POLLING_TIMEOUT(tMALTab[memIdx]->EraseTimeout);
        }

        return MAL_OK;
    }
    else
    {
        return MAL_FAIL;
    }
}

/**
  * @brief  Check the address is supported
  * @param  Addr: Sector address/code (allow to determine which memory will be addressed)
  * @retval Index of the addressed memory
  */
static uint8_t  IAP_MAL_CheckAddr (uint32_t Addr)
{
    uint32_t memIdx = 0;

    /* Check with all supported memories */
    for(memIdx = 0; memIdx < MAX_USED_MEMORY_MEDIA; memIdx++)
    {
        /* If the check address is supported, return the memory index */
        if (tMALTab[memIdx]->pMAL_CheckAdd(Addr) == MAL_OK)
        {
            return memIdx;
        }
    }

    /* If there is no memory found, return MAX_USED_MEDIA */
    return (MAX_USED_MEMORY_MEDIA);
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
