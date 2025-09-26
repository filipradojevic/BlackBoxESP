/**
 * @file    lfs_flash.h
 * @brief   .
 * @version	1.0.0
 * @date    19.09.2025
 * @author  BetaTehPro  
 */


#ifndef LFS_FLASH_H
#define LFS_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lfs.h"

/* External hardware drivers */
#include "flash.h"
#include "gd5f2gq5ue.h"

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
 * API
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
                           size_t file_buffer_size);


 /**
 * @brief Read from Flash backend for LittleFS.
 *
 * @param c      LittleFS config (context should point to flash device)
 * @param block  Block index
 * @param off    Byte offset within block
 * @param buffer Destination buffer
 * @param size   Number of bytes to read
 * @return LFS_ERR_OK on success, LFS error code on failure
 */
int _flash_lfs_bd_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                       void *buffer, lfs_size_t size);

/**
 * @brief Program (write) to Flash backend for LittleFS.
 *
 * @param c      LittleFS config (context should point to flash device)
 * @param block  Block index
 * @param off    Byte offset within block
 * @param buffer Source buffer
 * @param size   Number of bytes to write
 * @return LFS_ERR_OK on success, LFS error code on failure
 */
int _flash_lfs_bd_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                       const void *buffer, lfs_size_t size);

/**
 * @brief Erase block on Flash backend for LittleFS.
 *
 * @param c     LittleFS config (context should point to flash device)
 * @param block Block index to erase
 * @return LFS_ERR_OK on success, LFS error code on failure
 */
int _flash_lfs_bd_erase(const struct lfs_config *c, lfs_block_t block);

/**
 * @brief Synchronize Flash backend (flush caches if required).
 *
 * @param c LittleFS config
 * @return LFS_ERR_OK on success, LFS error code on failure
 */
int _flash_lfs_bd_sync(const struct lfs_config *c);



#ifdef __cplusplus
}
#endif

#endif /* LFS_FLASH_H */