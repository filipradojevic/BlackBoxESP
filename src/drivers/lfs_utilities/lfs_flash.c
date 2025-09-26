/**
 * @file    lfs_flash.c
 * @brief   .
 * @version	1.0.0
 * @date    19.09.2025
 * @author  BetaTehPro  
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "lfs_flash.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void flash_lfs_init_config(struct lfs_config *cfg,
                           struct lfs_file_config *fcfg,
                           flash_t *flash_dev,
                           uint8_t* read_buffer,
                           size_t read_buffer_size,
                           uint8_t* prog_buffer,
                           size_t prog_buffer_size,
                           uint8_t* lookahead_buffer,
                           size_t lookahead_buffer_size,
                           uint8_t* file_buffer,
                           size_t file_buffer_size)
{
    memset(cfg, 0, sizeof(*cfg));
    memset(fcfg, 0, sizeof(*fcfg));

    cfg->context        = flash_dev;
    cfg->read           = _flash_lfs_bd_read;
    cfg->prog           = _flash_lfs_bd_prog;
    cfg->erase          = _flash_lfs_bd_erase;
    cfg->sync           = _flash_lfs_bd_sync;

    /* use sizes passed in, not sizeof(pointer) */
    cfg->read_size      = read_buffer_size;
    cfg->prog_size      = prog_buffer_size;
    cfg->block_size     = flash_dev->geometry.blkSize;
    cfg->block_count    = flash_dev->geometry.blkCnt;
    cfg->block_cycles   = -1;
    cfg->cache_size     = file_buffer_size;
    cfg->lookahead_size = lookahead_buffer_size;
    cfg->compact_thresh = -1;

    cfg->read_buffer      = read_buffer;
    cfg->prog_buffer      = prog_buffer;
    cfg->lookahead_buffer = lookahead_buffer;

    fcfg->attrs      = NULL;
    fcfg->attr_count = 0;
    fcfg->buffer     = file_buffer;
}



int _flash_lfs_bd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 void *buffer, lfs_size_t size)
{
    /* basic sanity checks */
    if ((c == NULL) || (buffer == NULL)) {
        return LFS_ERR_IO;
    }

    /* bounds check: offset + size must fit in block */
    if (((uint64_t)off + (uint64_t)size) > (uint64_t)c->block_size) {
        return LFS_ERR_INVAL;
    }

    flash_t *flash = (flash_t *)c->context;
    uint32_t block_addr = (uint32_t)block;
    uint32_t page_offset = (uint32_t)off;

    /* use configured read_size from lfs_config */
    uint32_t read_unit = (uint32_t)c->read_size;
    uint32_t page = page_offset / read_unit;

    if (FLASH_PageRead(flash, block_addr, page, (uint8_t *)buffer, (uint32_t)size) != ESP_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}


int _flash_lfs_bd_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                 const void *buffer, lfs_size_t size)
{
    if ((c == NULL) || (buffer == NULL)) {
        return LFS_ERR_IO;
    }

    /* bounds check */
    if (((uint64_t)off + (uint64_t)size) > (uint64_t)c->block_size) {
        return LFS_ERR_INVAL;
    }

    flash_t *flash = (flash_t *)c->context;
    uint32_t blkAddr = (uint32_t)block;

    /* use lfs prog_size / page unit */
    uint32_t prog_unit = (uint32_t)c->prog_size;
    uint32_t pageAddr = (uint32_t)off / prog_unit;
    uint32_t column   = (uint32_t)off % prog_unit;

    if ((column + (uint32_t)size) > prog_unit) {
        return LFS_ERR_INVAL;
    }

    // FLASH_PageWrite(flash, blkAddr, pageAddr, (uint8_t*)buffer, (uint32_t)size);

    if (GD5F2GQ5UE_CacheWrite(flash, column, (uint8_t*)buffer, (uint32_t)size, FLASH_DMA) != ESP_OK) {
        return LFS_ERR_IO;
    }

    if (GD5F2GQ5UE_CacheFlush(flash, blkAddr, pageAddr, FLASH_WAIT_EXECUTION) != ESP_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

int _flash_lfs_bd_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (c == NULL) {
        return LFS_ERR_IO;
    }

    flash_t *flash = (flash_t *)c->context;
    uint32_t block_addr = (uint32_t)block;

    if (FLASH_BlockErase(flash, block_addr, 0) != ESP_OK) {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

int _flash_lfs_bd_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}