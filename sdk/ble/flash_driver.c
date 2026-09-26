/**
  ******************************************************************************
  * @file    flash_driver.c
  * @author  MCD Application Team
  * @brief   The Flash Driver module is the interface layer between Flash
  *          management modules and HAL Flash drivers
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_driver.h"
#include "utilities_conf.h"
/* tiles: this SDK has no ST HAL; the erase and quad-word program are the
 * ll_flash.h ones (RM0493 §7.3.6 / §7.3.7), single bank. Changes marked. */
#include "ll_flash.h"

/* Global variables ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define FD_CTRL_NO_BIT_SET   (0UL) /* value used to reset the Flash Control status */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief variable used to represent the Flash Control status
 */
static volatile FD_Flash_ctrl_bm_t FD_Flash_Control_status = FD_CTRL_NO_BIT_SET;

/**
 * @brief tiles: set when the flash itself reported an error (FLASH_NSSR), as
 *        opposed to access being refused outside a time window
 */
static volatile uint8_t FD_HwError = 0u;

/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Update Flash Control status
  * @param  Flags_bm: Bit mask identifying the caller (1 bit per user)
  * @param  Status:   Action requested (enable or disable flash access)
  * @retval None
  */
void FD_SetStatus(FD_Flash_ctrl_bm_t Flags_bm, FD_FLASH_Status_t Status)
{
  UTILS_ENTER_CRITICAL_SECTION();

  switch (Status)
  {
    case LL_FLASH_DISABLE:
    {
      FD_Flash_Control_status |= (1u << Flags_bm);
      break;
    }
    case LL_FLASH_ENABLE:
    {
      FD_Flash_Control_status &= ~(1u << Flags_bm);
      break;
    }
    default :
    {
      break;
    }
  }

  UTILS_EXIT_CRITICAL_SECTION();
}

/**
  * @brief  Write a block of 128 bits (4 32-bit words) in Flash
  * @param  Dest: Address where to write in Flash (128-bit aligned)
  * @param  Payload: Address of data to be written in Flash (32-bit aligned)
  * @retval FD_FlashOp_Status_t: Success or failure of Flash write operation
  */
FD_FlashOp_Status_t FD_WriteData(uint32_t Dest, uint32_t Payload)
{
  FD_FlashOp_Status_t status = FD_FLASHOP_FAILURE;

  /* Check if RFTS OR Application allow flash access */
  if ((FD_Flash_Control_status & (1u << FD_FLASHACCESS_RFTS)) &&
      (FD_Flash_Control_status & (1u << FD_FLASHACCESS_RFTS_BYPASS)))
  { /* Access not allowed */
    return status;
  }

  /* Wait for system to allow flash access */
  while (FD_Flash_Control_status & (1u << FD_FLASHACCESS_SYSTEM));

  /* tiles: stop at the first hardware error (see FD_HwErrorPending) */
  if (FD_HwError != 0u)
  {
    return status;
  }

  if (ll_flash_program_qword(Dest, (const uint32_t *)Payload) == 0)
  {
    status = FD_FLASHOP_SUCCESS;
  }
  else
  {
    FD_HwError = 1u;
  }
  return status;
}

/**
  * @brief  Erase one sector of Flash
  * @param  Sect: Identifier of the sector to erase
  * @retval FD_FlashOp_Status_t: Success or failure of Flash erase operation
  */
FD_FlashOp_Status_t FD_EraseSectors(uint32_t Sect)
{
  FD_FlashOp_Status_t status = FD_FLASHOP_FAILURE;

  /* tiles: the WBA55 is single bank, 128 pages (FLASH_PAGE_COUNT) */
  if (FLASH_PAGE_COUNT <= Sect)
  {
    return status;
  }

  /* Check if LL allows flash access */
  if ((FD_Flash_Control_status & (1u << FD_FLASHACCESS_RFTS)) &&
      (FD_Flash_Control_status & (1u << FD_FLASHACCESS_RFTS_BYPASS)))
  { /* Access not allowed */
    return status;
  }

  /* Wait for system to allow flash access */
  while (FD_Flash_Control_status & (1u << FD_FLASHACCESS_SYSTEM));

  if (FD_HwError != 0u)
  {
    return status;
  }

  if (ll_flash_erase_page(Sect) == 0)
  {
    status = FD_FLASHOP_SUCCESS;
  }
  else
  {
    FD_HwError = 1u;
  }

  return status;
}

/**
  * @brief  tiles: whether a flash hardware error is waiting to be collected
  * @param  None
  * @retval 1 if FD_WriteData / FD_EraseSectors failed in the flash itself
  */
uint8_t FD_HwErrorPending(void)
{
  return FD_HwError;
}

/**
  * @brief  tiles: collect (and clear) the flash hardware error flag
  * @param  None
  * @retval 1 if an operation since the last call failed in the flash itself
  */
uint8_t FD_TakeHwError(void)
{
  uint8_t e;
  UTILS_ENTER_CRITICAL_SECTION();
  e = FD_HwError;
  FD_HwError = 0u;
  UTILS_EXIT_CRITICAL_SECTION();
  return e;
}
