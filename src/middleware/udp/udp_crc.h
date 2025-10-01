/**
 * @file    udp_crc.h
 * @brief   Library used to calculate CRC needed for UDP communication.
 * @version 1.0.0
 * @date    28.01.2024
 * @author  LisumLab
 */

#ifndef UDP_CRC_H
#define UDP_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

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

/**
 * @brief 32-bit CRC for ethernet checksum.
 *
 * @param[in] crc   start CRC value.
 * @param[in] mem   pointer to array.
 * @param[in] len   array length.
 * @return ETH CRC32
 */
uint32_t udp_crc32_eth(uint32_t crc, void const *mem, uint16_t len);

/**
 * @brief IPv4 header checksum.
 *
 * @param[in] data  pointer to array.
 * @param[in] len   array length.
 * @return CRC16 of IPv4 header
 */
uint16_t udp_crc16_ip(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UDP_CRC_H */