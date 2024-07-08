/**
  ******************************************************************************
  * @file    flash_l4.c
  * @author  MCD Application Team
  * @brief   Management of the L4 internal flash memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include <ubinos/ubidrv/nvmem.h>

#if (UBINOS__UBIDRV__INCLUDE_NVMEM == 1)
#if (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NRF52840DK)

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "bsp.h"

#include <nrfx.h>
#include <nrfx_nvmc.h>

#undef LOGM_CATEGORY
#define LOGM_CATEGORY LOGM_CATEGORY__NVMEM

#define NVMEM_BASE (UBINOS__BSP__LINK_MEMMAP_FLASH_ORIGIN)
#define NVMEM_SIZE (UBINOS__BSP__LINK_MEMMAP_FLASH_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH2_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH3_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH4_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH5_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH6_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH7_LENGTH \
    + UBINOS__BSP__LINK_MEMMAP_FLASH8_LENGTH \
)

#define ROUND_DOWN(a,b) (((a) / (b)) * (b))

static uint32_t nvmem_get_page(uint8_t *addr)
{
    uint32_t page_size = nrfx_nvmc_flash_page_size_get();
    uint32_t page = 0U;

    page = ((uint32_t) addr - NVMEM_BASE) / page_size;

    return page;
}

ubi_err_t nvmem_erase(uint8_t *addr, size_t size)
{
    ubi_err_t ubi_err;

    uint32_t page;
    uint32_t page_addr;
    uint32_t page_size = nrfx_nvmc_flash_page_size_get();
    uint32_t end_page = nvmem_get_page((uint8_t *) ((uint32_t) addr + size - 1));

    do
    {
        if (addr < NVMEM_BASE)
        {
            ubi_err = UBI_ERR_ERROR;
            break;
        }
        if (size <= 0)
        {
            ubi_err = UBI_ERR_ERROR;
            break;
        }

        page = nvmem_get_page(addr);
        do
        {
            page_addr = NVMEM_BASE + (page * page_size);
            nrf_nvmc_page_erase(page_addr);
            page++;
            if (page > end_page)
            {
                break;
            }
        } while (1);

        ubi_err = UBI_ERR_OK;
        break;
    } while (1);

    return ubi_err;
}

ubi_err_t nvmem_update(uint8_t *addr, const uint8_t *buf, size_t size)
{
    ubi_err_t ubi_err;

    uint32_t page_size = nrfx_nvmc_flash_page_size_get();
    uint32_t dst_addr = (uint32_t) addr;

    int remaining = size;
    uint8_t * src_addr = (uint8_t *) buf;
    uint8_t * page_cache = (uint8_t*) malloc(page_size);

    if(page_cache == NULL)
    {
        return UBI_ERR_NO_MEM;
    }

    do {
        uint32_t fl_addr = ROUND_DOWN(dst_addr, page_size);
        uint32_t fl_offset = dst_addr - fl_addr;
        uint32_t len = MIN(page_size - fl_offset, size);

        /* Load from the flash into the cache */
        memcpy(page_cache, (uint8_t *) fl_addr, page_size);
        /* Update the cache from the source */
        memcpy(page_cache + fl_offset, src_addr, len);
        /* Erase the page, and write the cache */

        ubi_err = nvmem_erase((uint8_t *) fl_addr, page_size);
        if (ubi_err != UBI_ERR_OK)
        {
            break;
        }

        nrf_nvmc_write_bytes(fl_addr, page_cache, page_size);

        dst_addr += len;
        src_addr += len;
        remaining -= len;

        if (remaining <= 0)
        {
            ubi_err = UBI_ERR_OK;
            break;
        }
    } while (1);

    free(page_cache);

    return ubi_err;
}

ubi_err_t nvmem_read(const uint8_t *addr, uint8_t *buf, size_t size)
{
    ubi_err_t ubi_err;

    do
    {
        memcpy((void *)buf, (void *)addr, size);
        ubi_err = UBI_ERR_OK;
    } while (0);

    ubi_err = UBI_ERR_OK;

    return ubi_err;
}

#endif /* (UBINOS__BSP__BOARD_MODEL == UBINOS__BSP__BOARD_MODEL__NRF52840DK) */
#endif /* (UBINOS__UBIDRV__INCLUDE_NVMEM == 1) */

