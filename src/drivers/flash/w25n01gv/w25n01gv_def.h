/**
 *	@file     w25n01gv_def.h
 *  @brief    W25N01GV NAND flash driver definitions.
 *  @version  v1.0.
 *  @author   LisumLab
 */

/* general device iniformation */
#define W25N01GV_JEDEC_ID            0xEFU
#define W25N01GV_DEVICE_ID           0xAA21U
#define W25N01GV_BLOCKS_PER_DEV      1024U
#define W25N01GV_PAGES_PER_BLOCK     64U
#define W25N01GV_PAGES_PER_DEV      (1024U * 64U)
#define W25N01GV_PAGE_SIZE           2048U
#define W25N01GV_SPARE_AREA_SIZE     64U
#define W25N01GV_SPARE_BLOCKS_NUM    20U
#define W25N01GV_BBM_LUT_ENTRY_CNT   20U

/* flash status registers */
#define W25N01GV_PROTECTION_REG_ADDR   0xA0
#define W25N01GV_CONFIG_REG_ADDR       0xB0
#define W25N01GV_STATUS_REG_ADDR       0xC0

/* status register 1 */
#define W25N01GV_PROTECTION_NONE   0x00

/* status register 2  */
#define W25N01GV_CONFIG_ENABLE_ECC       (1 << 4)
#define W25N01GV_CONFIG_BUFFER_READ_MODE (1 << 3)

/* status register 3 */
#define W25N01GV_STATUS_REGISTER_DEV_BUSY_MASK (1 << 0)
#define W25N01GV_STATUS_REGISTER_WRITE_EN_MASK (1 << 1)
#define W25N01GV_STATUS_REGISTER_ECC0_MASK     (1 << 4)
#define W25N01GV_STATUS_REGISTER_ECC1_MASK     (1 << 5)

#define W25N01GV_ECC_STATUS_SUCCESS1 0x00U
#define W25N01GV_ECC_STATUS_SUCCESS2 0x01U

/* bad block managment masks */

#define W25N01GV_BBM_LUT_LINK_STATUS1  (1 << 15)
#define W25N01GV_BBM_LUT_LINK_STATUS2  (1 << 14)
#define W25N01GV_BBMLUT_LINK_STATUS_OFFSET  14

/* timing definitions */
#define W25N01GV_DEV_RESET_TIMEOUT_US    500     /*<- tRST */
#define W25N01GV_PAGE_PROGRAM_TIMEOUT_US 700     /*<- tPP */
#define W25N01GV_PAGE_READ_TIMEOUT_US    60      /*<- tRD2 */
#define W25N01GV_BLOCK_ERASE_TIMEOUT_US  10000   /*<- tBE */

/* flash commands */
#define W25N01GV_PROGRAM_LOAD_CMD         0x02U
#define W25N01GV_READ_DATA_CMD            0x03U
#define W25N01GV_WRITE_DISABLE_CMD        0x04U
#define W25N01GV_WRITE_ENABLE_CMD         0x06U
#define W25N01GV_READ_FAST_CMD            0x0BU
#define W25N01GV_GET_FEATURE_CMD          0x0FU
#define W25N01GV_PROGRAM_EXECUTE_CMD      0x10U
#define W25N01GV_PAGE_DATA_READ_CMD       0x13U
#define W25N01GV_SET_FEATURE_CMD          0x1FU
#define W25N01GV_PROGRAM_LOAD_RANDOM_CMD  0x84U
#define W25N01GV_ADD_TO_BBM_LUT_CMD       0xA1U
#define W25N01GV_READ_BBM_LUT_CMD         0xA5U
#define W25N01GV_BLOCK_ERASE_CMD          0xD8U
#define W25N01GV_DEV_RESET_CMD            0xFFU

#define W25N01GV_WRITE_EN_RETRY 20U