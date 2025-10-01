/**
 * @file    usd_def.h
 * @brief   SDHC/SDXC driver common types and definitions
 * @version	1.0.0
 * @date    26.02.2025
 * @author  LisumLab
 */

#ifndef SD_DEF_H
#define SD_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* commands */
#define SD_DEF_CMD0 0
#define SD_DEF_CMD1 1
#define SD_DEF_CMD5 5
#define SD_DEF_CMD6 6
#define SD_DEF_CMD8 8
#define SD_DEF_CMD9 9
#define SD_DEF_CMD10 10
#define SD_DEF_CMD12 12
#define SD_DEF_CMD13 13
#define SD_DEF_CMD16 16
#define SD_DEF_CMD17 17
#define SD_DEF_CMD18 18
#define SD_DEF_CMD24 24
#define SD_DEF_CMD25 25
#define SD_DEF_CMD27 27
#define SD_DEF_CMD28 28
#define SD_DEF_CMD29 29
#define SD_DEF_CMD30 30
#define SD_DEF_CMD32 32
#define SD_DEF_CMD33 33
#define SD_DEF_CMD34 34
#define SD_DEF_CMD35 35
#define SD_DEF_CMD36 36
#define SD_DEF_CMD37 37
#define SD_DEF_CMD38 38
#define SD_DEF_CMD42 42
#define SD_DEF_CMD50 50
#define SD_DEF_CMD52 52
#define SD_DEF_CMD53 53
#define SD_DEF_CMD55 55
#define SD_DEF_CMD56 56
#define SD_DEF_CMD57 57
#define SD_DEF_CMD58 58
#define SD_DEF_CMD59 59
#define SD_DEF_ACMD13 13
#define SD_DEF_ACMD22 22
#define SD_DEF_ACMD23 23
#define SD_DEF_ACMD41 41
#define SD_DEF_ACMD42 42
#define SD_DEF_ACMD51 51

/*
 * command checksums
 *
 * The SPI interface is initialized in the CRC OFF mode in default. However, the
 * RESET command (CMD0) that is used to switch the card to SPI mode, is received
 * by the card while in SD mode and, therefore, shall have a valid CRC field.
 *
 * Also, The CMD8 CRC verification is always enabled. The Host shall set correct
 * CRC in the argument of CMD8. If CRC error is detected, card returns CRC error
 * in R1 response regardless of command index.
 *
 * SD Specifications (Part 1) - Physical Layer Simplified Specification
 * Version 9.10 - page 289 of 383
 */
#define SD_DEF_CMD0_CRC 0x4A
#define SD_DEF_CMD8_CRC 0x43

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SD_DEF_H */