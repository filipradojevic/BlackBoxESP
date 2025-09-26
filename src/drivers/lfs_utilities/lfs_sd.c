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

void sd_lfs_init_config(struct lfs_config* cfg, struct lfs_file_config* fcfg,
						sd_t* sd_dev, uint8_t* read_buffer,
						size_t read_buffer_size, uint8_t* prog_buffer,
						size_t prog_buffer_size, uint8_t* lookahead_buffer,
						size_t lookahead_buffer_size, uint8_t* file_buffer,
						size_t file_buffer_size)

{
	memset(cfg, 0, sizeof(*cfg));
	memset(fcfg, 0, sizeof(*fcfg));

	cfg->context = sd_dev;
	cfg->read = _sd_lfs_bd_read;
	cfg->prog = _sd_lfs_bd_prog;
	cfg->erase = _sd_lfs_bd_erase;
	cfg->sync = _sd_lfs_bd_sync;

	cfg->read_size = read_buffer_size;
	cfg->prog_size = prog_buffer_size;
	cfg->block_size = sd_dev->info.blk_len << LFS_BD_SHIFT;
	cfg->block_count = sd_dev->info.blk_cnt >> LFS_BD_SHIFT;
	cfg->block_cycles = -1;
	cfg->cache_size = file_buffer_size;
	cfg->lookahead_size = lookahead_buffer_size;
	cfg->compact_thresh = -1;

	cfg->read_buffer = read_buffer;
	cfg->prog_buffer = prog_buffer;
	cfg->lookahead_buffer = lookahead_buffer;

	fcfg->attrs = NULL;
	fcfg->attr_count = 0;
	fcfg->buffer = file_buffer;
}

int _sd_lfs_bd_read(const struct lfs_config* c, lfs_block_t block,
					lfs_off_t off, void* buffer, lfs_size_t size)
{
	sd_t* sd = (sd_t*)c->context;

	/* wait for card to get out of the busy state */
	while (sd_busy(sd))
		;

	uint32_t addr = (block << LFS_BD_SHIFT) + (off / sd->info.blk_len);

	/* read block of data */
	if (sd_blk_read(sd, addr, buffer))
		return LFS_ERR_IO;

	return LFS_ERR_OK;
}

int _sd_lfs_bd_prog(const struct lfs_config* c, lfs_block_t block,
					lfs_off_t off, const void* buffer, lfs_size_t size)
{
	sd_t* sd = (sd_t*)c->context;

	/* wait for card to get out of the busy state */
	while (sd_busy(sd))
		;

	uint32_t addr = (block << LFS_BD_SHIFT) + (off / sd->info.blk_len);

	/* write block of data */
	if (sd_blk_write(sd, addr, (uint8_t*)(buffer)))
		return LFS_ERR_IO;

	return LFS_ERR_OK;
}

int _sd_lfs_bd_erase(const struct lfs_config* c, lfs_block_t block)
{
	/* sd card already handles block erasure */
	return LFS_ERR_OK;
}

int _sd_lfs_bd_sync(const struct lfs_config* c)
{
	/* sd card does not require sync */
	return LFS_ERR_OK;
}
