/**
 *	@file     gd5f2gq5ue_def.h
 *  @brief    GD5F2GQ5UE NAND flash driver definitions.
 *  @version  v1.0.
 *  @author   LisumLab
 */

/* general device iniformation */
#define GD5F2GQ5UE_JEDEC_ID            0xC8U
#define GD5F2GQ5UE_DEVICE_ID           0x52U
#define GD5F2GQ5UE_BLOCKS_PER_DEV      2048U
#define GD5F2GQ5UE_PAGES_PER_BLOCK     64U
#define GD5F2GQ5UE_PAGES_PER_DEV      (2048U * 64U)
#define GD5F2GQ5UE_PAGE_SIZE           2048U
#define GD5F2GQ5UE_SPARE_AREA_SIZE     128U
#define GD5F2GQ5UE_SPARE_BLOCKS_NUM    40U

/* flash registers */
#define GD5F2GQ5UE_PROTECTION_REG_ADDR  0xA0
#define GD5F2GQ5UE_FEATURE_1_REG_ADDR   0xB0
#define GD5F2GQ5UE_STATUS_1_REG_ADDR    0xC0
#define GD5F2GQ5UE_FEATURE_2_REG_ADDR   0xD0
#define GD5F2GQ5UE_STATUS_2_REG_ADDR    0xF0

/* protection register */
#define GD5F2GQ5UE_PROTECTION_NONE   0x00

#define GD5F2GQ5UE_PROTECTION_BRWD  (1 << 7)
#define GD5F2GQ5UE_PROTECTION_BP2   (1 << 5)
#define GD5F2GQ5UE_PROTECTION_BP1   (1 << 4)
#define GD5F2GQ5UE_PROTECTION_BP0   (1 << 3)
#define GD5F2GQ5UE_PROTECTION_INV   (1 << 2)
#define GD5F2GQ5UE_PROTECTION_CMP   (1 << 1)

/* feature register 1 */
#define GD5F2GQ5UE_FEATURE_1_OTP_PRT  (1 << 7)
#define GD5F2GQ5UE_FEATURE_1_OTP_EN   (1 << 6)
#define GD5F2GQ5UE_FEATURE_1_ECC_EN   (1 << 4)
#define GD5F2GQ5UE_FEATURE_1_QE       (1 << 0)

/* status register 1 */
#define GD5F2GQ5UE_STATUS_1_ECCS1   (1 << 5)
#define GD5F2GQ5UE_STATUS_1_ECCS0   (1 << 4)
#define GD5F2GQ5UE_STATUS_1_P_FAIL  (1 << 3)
#define GD5F2GQ5UE_STATUS_1_E_FAIL  (1 << 2)
#define GD5F2GQ5UE_STATUS_1_WEL     (1 << 1)
#define GD5F2GQ5UE_STATUS_1_OIP     (1 << 0)

/* feature register 2 */
#define GD5F2GQ5UE_FEATURE_2_DS_IO_1  (1 << 6)
#define GD5F2GQ5UE_FEATURE_2_DS_IO_0  (1 << 5)

/* status register 2 */
#define GD5F2GQ5UE_STATUS_2_ECCSE1  (1 << 5)
#define GD5F2GQ5UE_STATUS_2_ECCSE0  (1 << 4)
#define GD5F2GQ5UE_STATUS_2_BPS     (1 << 3)
#define GD5F2GQ5UE_STATUS_2_CBSY    (1 << 0)


/* timing definitions */
#define GD5F2GQ5UE_DEV_RESET_TIMEOUT_US    2500    /*<- tRST */
#define GD5F2GQ5UE_PAGE_PROGRAM_TIMEOUT_US 2500   /*<- tPROG */ //this is increased bcs of speed of logging
#define GD5F2GQ5UE_PAGE_READ_TIMEOUT_US    400     /*<- tRD */
#define GD5F2GQ5UE_BLOCK_ERASE_TIMEOUT_US  10000   /*<- tBERS */

/* flash commands */
#define GD5F2GQ5UE_PROGRAM_LOAD_CMD         0x02U
#define GD5F2GQ5UE_READ_DATA_CMD            0x03U
#define GD5F2GQ5UE_WRITE_DISABLE_CMD        0x04U
#define GD5F2GQ5UE_WRITE_ENABLE_CMD         0x06U
#define GD5F2GQ5UE_GET_FEATURE_CMD          0x0FU
#define GD5F2GQ5UE_PROGRAM_EXECUTE_CMD      0x10U
#define GD5F2GQ5UE_PAGE_READ_TO_CACHE_CMD   0x13U
#define GD5F2GQ5UE_SET_FEATURE_CMD          0x1FU
#define GD5F2GQ5UE_NEXT_PAGE_TO_CACHE_CMD   0x31U
#define GD5F2GQ5UE_LAST_PAGE_CACHE_CMD      0x3FU
#define GD5F2GQ5UE_PROGRAM_LOAD_RANDOM_CMD  0x84U
#define GD5F2GQ5UE_BLOCK_ERASE_CMD          0xD8U
#define GD5F2GQ5UE_DEV_RESET_CMD            0xFFU

#define GD5F2GQ5UE_WRITE_EN_RETRY           20U