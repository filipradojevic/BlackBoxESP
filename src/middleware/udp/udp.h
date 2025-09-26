/**
 * @file    udp.h
 * @brief   Library used to format, transmit and receive UDP packets.
 * @version 1.0.0
 * @date    28.01.2024
 * @author  LisumLab
 */

#ifndef UDP_H
#define UDP_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>

#ifdef UDP_CONFIG
#include "udp_config.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* 0 for big-endian and 1 for little-endian architectures */
#ifndef UDP_ENDIANNESS
#define UDP_ENDIANNESS 1
#endif
/* maximum size of ethernet packet */
#ifndef UDP_ETH_MAX
#define UDP_ETH_MAX 1536
#endif
/* set if ethernet checksum should be appended on send */
#ifndef UDP_ETH_CRC_SEND
#define UDP_ETH_CRC_SEND 0
#endif
/* set if ethernet checksum should be validated on recv */
#ifndef UDP_ETH_CRC_RECV
#define UDP_ETH_CRC_RECV 0
#endif

/* maximum number of entries in ARP table */
#ifndef UDP_ARP_MAX
#define UDP_ARP_MAX 16
#endif
/* set if gratuitous ARP support is enabled */
#ifndef UDP_ARP_GRAT
#define UDP_ARP_GRAT 1
#endif

/* maximum number of UDP connections to track */
#ifndef UDP_TRACK_MAX
#define UDP_TRACK_MAX 8
#endif

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

typedef struct udp_t udp_t;

/*! @brief interface to be implemented by physical layer */
typedef struct udp_phy_t {
	/* read data from the device */
	uint16_t (*recv)(void *phy, uint8_t *data, uint16_t size);
	/* write data to the device */
	uint16_t (*send)(void *phy, const uint8_t *data, uint16_t size);
} udp_phy_t;

/*! @brief connection tracking parameters and data */
typedef struct udp_track_t {
	/* MAC address */
	uint8_t mac[6];
	/* IP address */
	uint8_t ip[4];
	/* UDP port */
	uint16_t port;

	/* callback on receiving from tracked connection */
	int (*recv_cb)(udp_t *udp, uint8_t *data, uint16_t size, void *arg);
	/* additional argument to receive callback */
	void *recv_cb_arg;
} udp_track_t;

/*! @brief table of static ARP entries */
typedef struct udp_arptab_t {
	/* MAC addresses of entries */
	uint8_t mac[UDP_ARP_MAX][6];
	/* IP addresses of entries */
	uint8_t ip[UDP_ARP_MAX][4];
	/* number of entries */
	uint8_t len;
} udp_arptab_t;

/*! @brief device parameters and data */
typedef struct udp_t {
	/* MAC address of the device */
	uint8_t mac[6];
	/* IP address of the device */
	uint8_t ip[4];

	/* table of static ARP entries */
	udp_arptab_t arptab;

	/* table of tracked connections */
	udp_track_t track[UDP_TRACK_MAX];
	/* number of tracked connections */
	uint8_t track_count;

	/* interface to physical layer */
	udp_phy_t *phy;

	/* internal device buffer */
	uint8_t buffer[UDP_ETH_MAX];
} udp_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize UDP.
 *
 * @param[in] udp   UDP device.
 * @param[in] mac   MAC address of the device.
 * @param[in] ip    IP address of the device.
 * @param[in] phy   Interface to physical layer.
 * @return None
 */
void udp_init(udp_t *udp, const uint8_t *mac, const uint8_t *ip,
			  udp_phy_t *phy);

/**
 * @brief   Add entry to static ARP table.
 * @note    After device initialization table is empty.
 *          At most UDP_ARP_MAX entries can be contained in the table.
 *          Function calls will return -1 when table is full.
 *
 * @param[in] udp   UDP device.
 * @param[in] mac   MAC address of the new entry.
 * @param[in] ip    IP address of the new entry.
 * @return New entry's index on success and -1 on error
 */
int udp_arptab_add(udp_t *udp, const uint8_t *mac, const uint8_t *ip);

/**
 * @brief   Track new udp connection.
 * @note 	IP can be set to 255.255.255.255 to recieve from all IPs
 *          Port can be set to 0 to receive on all ports
 *          Function calls will return -1 if there is no entry in the ARP table
 *          for IP
 *
 *  callback specification:
 *
 *  int (*recv_cb)(udp_t *udp, uint8_t *data, uint16_t size, void *arg);
 *      udp     UDP device
 *      data    UDP packet data
 *      size    size of UDP packet
 *      arg     additional argument
 *      return value is 1 for correctly processed packet and -1 on error
 *
 * @param[in] udp           UDP device.
 * @param[in] ip            IP address.
 * @param[in] port          UDP port.
 * @param[in] recv_cb       callback triggered on receiving new packet.
 * @param[in] recv_cb_arg   additional argument to receive callback
 * @return Index of new connection on success and -1 on error
 */
int udp_track(udp_t *udp, const uint8_t *ip, uint16_t port,
			  int (*recv_cb)(udp_t *udp, uint8_t *data, uint16_t size,
							 void *arg),
			  void *recv_cb_arg);

/**
 * @brief   Process UDP communication.
 * @details Processes UDP connections and communicates with physical interface
 *          If UDP_PING_REPLY is set, function will also reply to ICMP ping
 *          requests.
 *
 * @param[in] udp   UDP device.
 * @return None
 */
void udp_process(udp_t *udp);

/**
 * @brief Send UDP packet.
 *
 * @param[in] udp   UDP device.
 * @param[in] ip    receipient IP address.
 * @param[in] port  receipient UDP port.
 * @param[in] data  UDP packet data.
 * @param[in] size  size of UDP packet.
 * @return Packet data size on success and -1 on error.
 */
uint16_t udp_send(udp_t *udp, const uint8_t *ip, uint16_t port,
				  const uint8_t *data, uint16_t size);

/**
 * @brief Helper functions for host-network order conversion.
 *
 * @param[in] value 16-bit or 32-bit value.
 * @return  Value is the value passed converted from host to network order
 *          and depends on UDP_ENDIANESS defition.
 */
uint16_t udp_reorder_16(uint16_t value);
uint32_t udp_reorder_32(uint32_t value);

/**
 * @brief Utility functions to calculate UDP packet size after packing/parsing.
 * @note udp_parse_size gives an upper bound of the size after parsing
 *
 * @param[in] size  packet size before packing/parsing.
 * @return packet size after packing/parsing.
 */
uint16_t udp_pack_size(uint16_t size);
uint16_t udp_parse_size(uint16_t size);

#if UDP_ARP_GRAT
/**
 * @brief Utility function to send out arp announcements to other devices.
 * @note function should be called periodically (e.g. every 10s)
 *
 * @param[in] udp UDP device.
 * @return None.
 */
void udp_arp_grat(udp_t *udp);
#endif

#ifdef __cplusplus
}
#endif

#endif /* UDP_H */