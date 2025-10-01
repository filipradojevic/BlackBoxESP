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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lfs.h"
#include "lfs_sd.h"

/* External hardware drivers */
#include "sd.h"

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

void sd_lfs_init_config(struct lfs_config* cfg,
                        struct lfs_file_config* fcfg,
                        sdmmc_card_t* sd,
                        uint8_t* read_buffer, size_t read_buffer_size,
                        uint8_t* prog_buffer, size_t prog_buffer_size,
                        uint8_t* lookahead_buffer, size_t lookahead_buffer_size,
                        uint8_t* file_buffer, size_t file_buffer_size)
{
    memset(cfg, 0, sizeof(*cfg));
    memset(fcfg, 0, sizeof(*fcfg));

    cfg->context = sd;
    cfg->read = _sd_lfs_bd_read;
    cfg->prog = _sd_lfs_bd_prog;
    cfg->erase = _sd_lfs_bd_erase;
    cfg->sync = _sd_lfs_bd_sync;

    // ESP-IDF sdmmc sektor je obično 512 bajta
    cfg->read_size = sd->csd.sector_size;
    cfg->prog_size = sd->csd.sector_size;
    cfg->block_size = sd->csd.sector_size;
    cfg->block_count = sd->csd.capacity; // broj sektora na kartici
    cfg->block_cycles = -1;

    cfg->cache_size = read_buffer_size; // može biti isto kao sektor
    cfg->lookahead_size = lookahead_buffer_size;
    cfg->compact_thresh = -1;

    cfg->read_buffer = read_buffer;
    cfg->prog_buffer = prog_buffer;
    cfg->lookahead_buffer = lookahead_buffer;

    fcfg->attrs = NULL;
    fcfg->attr_count = 0;
    fcfg->buffer = file_buffer;
}

int _sd_lfs_bd_read(const struct lfs_config* c,
                    lfs_block_t block, lfs_off_t off,
                    void* buffer, lfs_size_t size)
{
    sdmmc_card_t* sd = (sdmmc_card_t*)c->context;

    esp_err_t ret = sdmmc_read_sectors(sd, buffer, block, 1);
    if (ret != ESP_OK) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

int _sd_lfs_bd_prog(const struct lfs_config* c,
                    lfs_block_t block, lfs_off_t off,
                    const void* buffer, lfs_size_t size)
{
    sdmmc_card_t* sd = (sdmmc_card_t*)c->context;

    esp_err_t ret = sdmmc_write_sectors(sd, buffer, block, 1);
    if (ret != ESP_OK) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

int _sd_lfs_bd_erase(const struct lfs_config* c, lfs_block_t block)
{
    // SD kartica sama radi erase – nije potrebno
    return LFS_ERR_OK;
}

int _sd_lfs_bd_sync(const struct lfs_config* c)
{
    // SD kartici ne treba sync
    return LFS_ERR_OK;
}
