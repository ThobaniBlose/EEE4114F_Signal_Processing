/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */
#include <string.h>
#include "ff_gen_drv.h"

/* SD card SPI driver functions defined in main.c */
extern uint8_t sd_init(void);
extern uint8_t sd_read_block(uint32_t block_addr, uint8_t *buf);
extern uint8_t sd_write_block(uint32_t block_addr, const uint8_t *buf);

static volatile DSTATUS Stat = STA_NOINIT;
/* USER CODE END DECL */

DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};

DSTATUS USER_initialize (BYTE pdrv)
{
  /* USER CODE BEGIN INIT */
  (void)pdrv;
  if (Stat == 0) return Stat;

  HAL_Delay(200);

  if (sd_init() == 0U) {
    Stat = 0;
  } else {
    Stat = STA_NOINIT;
  }
  return Stat;
  /* USER CODE END INIT */
}

DSTATUS USER_status (BYTE pdrv)
{
  /* USER CODE BEGIN STATUS */
  (void)pdrv;
  return Stat;
  /* USER CODE END STATUS */
}

DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN READ */
  (void)pdrv;
  if (buff == NULL || count == 0U) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  for (UINT i = 0U; i < count; i++) {
    if (sd_read_block((uint32_t)(sector + i), &buff[i * 512U]) != 0U) {
      return RES_ERROR;
    }
  }
  return RES_OK;
  /* USER CODE END READ */
}

#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN WRITE */
  (void)pdrv;
  if (buff == NULL || count == 0U) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  for (UINT i = 0U; i < count; i++) {
    if (sd_write_block((uint32_t)(sector + i), &buff[i * 512U]) != 0U) {
      return RES_ERROR;
    }
  }
  return RES_OK;
  /* USER CODE END WRITE */
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
  /* USER CODE BEGIN IOCTL */
  (void)pdrv;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd) {
  case CTRL_SYNC:
    return RES_OK;
  case GET_SECTOR_SIZE:
    if (buff == NULL) return RES_PARERR;
    *(WORD*)buff = 512U;
    return RES_OK;
  case GET_BLOCK_SIZE:
    if (buff == NULL) return RES_PARERR;
    *(DWORD*)buff = 1U;
    return RES_OK;
  case GET_SECTOR_COUNT:
    if (buff == NULL) return RES_PARERR;
    *(DWORD*)buff = 0x100000UL;
    return RES_OK;
  default:
    return RES_PARERR;
  }
  /* USER CODE END IOCTL */
}
#endif
