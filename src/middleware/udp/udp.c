/**
 * @file    udp.c
 * @brief   Library used to format, transmit and receive UDP packets.
 * @version 1.0.0
 * @date    28.01.2024
 * @author  LisumLab
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdint.h>
#include <string.h>

#include "udp.h"
#include "udp_crc.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*! @brief ethernet packet type */
typedef enum udp_eth_type_t {
	UDP_ETH_TYPE_IPV4 = 0x0800,
	UDP_ETH_TYPE_ARP = 0x0806
} udp_eth_type_t;

/*! @brief IP protocol type */
typedef enum udp_ip_proto_t {
	UDP_IP_PROTO_ICMP = 0x01,
	UDP_IP_PROTO_UDP = 0x11
} udp_ip_proto_t;

/*! @brief ICMP message type */
typedef enum udp_icmp_type_t {
	UDP_ICMP_TYPE_ECHO_REPLY = 0x00,
	UDP_ICMP_TYPE_ECHO_REQUEST = 0x08
} udp_icmp_type_t;

#ifdef UDP_ARP_GRAT
/*! @brief ARP message type */
typedef enum udp_arp_type_t {
	UDP_ARP_TYPE_REQUEST = 0x01,
	UDP_ARP_TYPE_REPLY = 0x02
} udp_arp_type_t;
#endif

/* protocol header definitions */
#pragma pack(1)

/*! @brief ethernet header */
typedef struct udp_eth_hdr_t {
	uint8_t mac_dst[6];
	uint8_t mac_src[6];
	uint16_t ethertype;
} udp_eth_hdr_t;

/*! @brief ethernet footer */
typedef struct udp_eth_ftr_t {
	uint32_t crc;
} udp_eth_ftr_t;

/*! @brief IPv4 header */
typedef struct udp_ip_hdr_t {
	uint8_t version_ihl;
	uint8_t dscp_ecn;
	uint16_t length;
	uint16_t ident;
	uint16_t flags_fragment;
	uint8_t ttl;
	uint8_t protocol;
	uint16_t crc;
	uint8_t ip_src[4];
	uint8_t ip_dst[4];
	/* IP options not included */
} udp_ip_hdr_t;

/*! @brief UDP header */
typedef struct udp_udp_hdr_t {
	uint16_t port_src;
	uint16_t port_dst;
	uint16_t length;
	uint16_t crc;
} udp_udp_hdr_t;

/*! @brief ICMP header */
typedef struct udp_icmp_hdr_t {
	uint8_t type;
	uint8_t code;
	uint16_t crc;
	uint16_t ident;
	uint16_t seq;
} udp_icmp_hdr_t;

#if UDP_ARP_GRAT
/*! @brief ARP header */
typedef struct udp_arp_hdr_t {
	uint16_t htype;
	uint16_t ptype;
	uint8_t hw_addrlen;
	uint8_t prot_addrlen;
	uint16_t oper;
	uint8_t hw_addr_send[6];
	uint8_t prot_addr_send[4];
	uint8_t hw_addr_targ[6];
	uint8_t prot_addr_targ[4];
} udp_arp_hdr_t;
#endif

#pragma pack()

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* definition of special addresses */
const uint8_t udp_ip_bcast[4] = {0xff, 0xff, 0xff, 0xff};
const uint8_t udp_eth_bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* static utility function to resolve MAC address */
static const uint8_t *udp_resolve(udp_t *udp, const uint8_t *ip);

/* static utility functions for striping and inserting protocol headers */
static uint8_t *udp_parse(udp_t *udp, udp_track_t *track, uint16_t *size);
static uint8_t *udp_pack(udp_t *udp, udp_track_t *track, uint16_t *size);

/* Static utility function for handling ARP request. */
static int udp_arp(udp_t *udp);

/* static utility function to handle ICMP ping reply */
static int udp_icmp(udp_t *udp, uint16_t size);

/* definitions of total packet prefix and suffix sizes for outgoing packets */
#define UDP_PREFIX_SIZE                                                        \
	(sizeof(udp_eth_hdr_t) + sizeof(udp_ip_hdr_t) + sizeof(udp_udp_hdr_t))
#if UDP_ETH_CRC_SEND
#define UDP_SUFFIX_SIZE (sizeof(udp_eth_ftr_t))
#else
#define UDP_SUFFIX_SIZE 0
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void udp_init(udp_t *udp, const uint8_t *mac, const uint8_t *ip, udp_phy_t *phy)
{
	memcpy(udp->mac, mac, sizeof(udp->mac));
	memcpy(udp->ip, ip, sizeof(udp->ip));

	udp->phy = phy;
	udp->arptab.len = 0;
	udp->track_count = 0;
}

int udp_arptab_add(udp_t *udp, const uint8_t *mac, const uint8_t *ip)
{
	udp_arptab_t *arptab;

	arptab = &udp->arptab;
	if (arptab->len >= UDP_ARP_MAX)
		return -1;

	memcpy(arptab->mac[arptab->len], mac, sizeof(arptab->mac[arptab->len]));
	memcpy(arptab->ip[arptab->len], ip, sizeof(arptab->ip[arptab->len]));
	arptab->len++;

	return arptab->len - 1;
}

int udp_track(udp_t *udp, const uint8_t *ip, uint16_t port,
			  int (*recv_cb)(udp_t *, uint8_t *, uint16_t, void *),
			  void *recv_cb_arg)
{
	udp_track_t *track;

	if (udp->track_count >= UDP_TRACK_MAX)
		return -1;

	track = &udp->track[udp->track_count];
	memcpy(track->ip, ip, 4);
	track->port = port;
	track->recv_cb = recv_cb;
	track->recv_cb_arg = recv_cb_arg;
	udp->track_count++;

	return udp->track_count - 1;
}

void udp_process(udp_t *udp)
{
	uint16_t packet_size;
	udp_track_t *track;
	udp_track_t tmp_track;
	const uint8_t *mac;
	uint8_t *payload;

	/* check if there is anything to receive */
	packet_size = udp->phy->recv(udp->phy, udp->buffer, UDP_ETH_MAX);
	if (packet_size == 0)
		return;

	/* check if packet is ICMP, if so, handle it internally */
	if (udp_icmp(udp, packet_size) != -1)
		return;

	/* check if packet is ARP request, if so, handle it internally */
	if (udp_arp(udp) != -1)
		return;

	/* check if packet is UDP */
	payload = udp_parse(udp, &tmp_track, &packet_size);
	if (payload == NULL)
		return;

	/* check if there are tracked connections for this packet */
	for (int i = 0; i < udp->track_count; i++) {
		track = &udp->track[i];

		/* update connection MAC address */
		mac = udp_resolve(udp, track->ip);
		if (mac == NULL)
			continue;
		memcpy(track->mac, mac, 6);

		/* filter by MAC, IP and port */
		if (memcmp(track->mac, tmp_track.mac, 6) != 0 &&
			memcmp(track->mac, udp_eth_bcast, 6) != 0)
			continue;
		if (memcmp(track->ip, tmp_track.ip, 4) != 0 &&
			memcmp(track->ip, udp_ip_bcast, 4) != 0)
			continue;
		//if (track->port != tmp_track.port && track->port != 0)
			//continue;

		/* call receive callback if it is defined */
		track->recv_cb(udp, payload, packet_size, track->recv_cb_arg);
	}
}

uint16_t udp_send(udp_t *udp, const uint8_t *ip, uint16_t port,
				  const uint8_t *data, uint16_t size)
{
	udp_track_t tmp_track;
	const uint8_t *mac;

	/* drop if size of packet is too large */
	if (udp_pack_size(size) > UDP_ETH_MAX)
		return 0;

	/* drop if MAC address cannot be resolved */
	mac = udp_resolve(udp, ip);
	if (mac == NULL)
		return 0;

	memcpy(&tmp_track.mac, mac, 6);
	memcpy(&tmp_track.ip, ip, 4);
	tmp_track.port = port;

	memcpy(udp->buffer + UDP_PREFIX_SIZE, data, size);

	udp_pack(udp, &tmp_track, &size);

	return udp->phy->send(udp->phy, udp->buffer, size);
}

inline uint16_t udp_reorder_16(uint16_t value)
{
	uint32_t ret;

#if UDP_ENDIANNESS
	ret = (value >> 8);
	ret |= (value << 8) & 0xFF00;
#else
	ret = value;
#endif

	return (uint16_t)ret;
}

inline uint32_t udp_reorder_32(uint32_t value)
{
	uint32_t ret;

#if UDP_ENDIANNESS
	ret = (value >> 24) & 0x000000FF;
	ret |= (value >> 8) & 0x0000FF00;
	ret |= (value << 8) & 0x00FF0000;
	ret |= (value << 24) & 0XFF000000;
#else
	ret = value;
#endif
	return ret;
}

uint16_t udp_pack_size(uint16_t size)
{
	uint16_t diff = UDP_PREFIX_SIZE + UDP_SUFFIX_SIZE;

	return size + diff;
}

uint16_t udp_parse_size(uint16_t size)
{
	uint16_t diff = UDP_PREFIX_SIZE + UDP_SUFFIX_SIZE;

	if (size > diff)
		return size - diff;
	else
		return 0;
}

#if UDP_ARP_GRAT
void udp_arp_grat(udp_t *udp)
{
	udp_eth_hdr_t *eth_hdr;
#if UDP_ETH_CRC_SEND
	udp_eth_ftr_t *eth_ftr;
	uint32_t eth_crc;
#endif
	udp_arp_hdr_t *arp_hdr;

	uint16_t size;

	/* fill ethernet frame header */
	eth_hdr = (udp_eth_hdr_t *)udp->buffer;
	memcpy(eth_hdr->mac_dst, udp_eth_bcast, sizeof(eth_hdr->mac_dst));
	memcpy(eth_hdr->mac_src, udp->mac, sizeof(eth_hdr->mac_src));
	eth_hdr->ethertype = udp_reorder_16(UDP_ETH_TYPE_ARP);

	/* fill ARP frame header */
	arp_hdr = (udp_arp_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t));
	arp_hdr->htype = udp_reorder_16(1);
	arp_hdr->ptype = udp_reorder_16(UDP_ETH_TYPE_IPV4);
	arp_hdr->hw_addrlen = 6;
	arp_hdr->prot_addrlen = 4;
	arp_hdr->oper = udp_reorder_16(UDP_ARP_TYPE_REQUEST);
	memcpy(arp_hdr->hw_addr_send, udp->mac, sizeof(udp->mac));
	memcpy(arp_hdr->prot_addr_send, udp->ip, sizeof(udp->ip));
	memcpy(arp_hdr->hw_addr_targ, udp_eth_bcast, sizeof(arp_hdr->hw_addr_targ));
	memcpy(arp_hdr->prot_addr_targ, udp->ip, sizeof(udp->ip));

	size = sizeof(udp_eth_hdr_t) + sizeof(udp_arp_hdr_t);

#if UDP_ETH_CRC_SEND
	/* calculate ethernet frame checksum */
	eth_ftr = (udp_eth_ftr_t *)(udp->buffer + size);
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc = udp_crc32_eth(eth_crc, udp->buffer, size);
	eth_ftr->crc = eth_crc;
	size += sizeof(udp_eth_ftr_t *);

#endif

	udp->phy->send(udp->phy, udp->buffer, size);
}
#endif

/****************************** static functions ******************************/

static const uint8_t *udp_resolve(udp_t *udp, const uint8_t *ip)
{
	/* check if broadcast */
	if (memcmp(udp_ip_bcast, ip, 4) == 0)
		return udp_eth_bcast;

	/* lookup address in the arp table */
	for (int i = 0; i < udp->arptab.len; i++) {
		if (memcmp(&udp->arptab.ip[i][0], ip, 4) == 0)
			return &udp->arptab.mac[i][0];
	}

	return NULL;
}

static uint8_t *udp_parse(udp_t *udp, udp_track_t *track, uint16_t *size)
{
	udp_eth_hdr_t *eth_hdr;
#if UDP_ETH_CRC_RECV
	udp_eth_ftr_t *eth_ftr;
#endif
	udp_ip_hdr_t *ip_hdr;
	udp_udp_hdr_t *udp_hdr;
	uint16_t ihl;
	uint16_t ip_crc;
#if UDP_ETH_CRC_RECV
	uint32_t eth_crc;
#endif

	uint8_t *parsed;

	/* extract ethernet frame header */
	eth_hdr = (udp_eth_hdr_t *)udp->buffer;

	/* check if packet is IPv4 packet */
	if (eth_hdr->ethertype != udp_reorder_16(UDP_ETH_TYPE_IPV4))
		return NULL;

	/* check if packet's destination MAC is this device */
	if (memcmp(eth_hdr->mac_dst, udp->mac, sizeof(udp->mac)) != 0 &&
		memcmp(eth_hdr->mac_dst, udp_eth_bcast, sizeof(udp->mac)) != 0)
		return NULL;

	/* extract IP header */
	ip_hdr = (udp_ip_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t));

	/* check if IP version is 4 */
	if (ip_hdr->version_ihl >> 4 != 4)
		return NULL;

	/* check if protocol is UDP */
	if (ip_hdr->protocol != UDP_IP_PROTO_UDP)
		return NULL;

	/* check if packet's destination IP is this device */
	if (memcmp(ip_hdr->ip_dst, udp->ip, sizeof(udp->ip)) != 0 &&
		memcmp(ip_hdr->ip_dst, udp_ip_bcast, sizeof(udp->ip) != 0))
		return NULL;

	/* extract header length */
	ihl = (ip_hdr->version_ihl & 0x0F) * 4;

	/* validate IP header checksum */
	ip_crc = udp_crc16_ip((const uint8_t *)ip_hdr, ihl);
	if (ip_crc != 0)
		return NULL;

	/* extract UDP header */
	udp_hdr = (udp_udp_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t) + ihl);

#if UDP_ETH_CRC_RECV
	/* validate ethernet checksum */
	eth_ftr = (udp_eth_ftr_t *)(udp->buffer + *size - sizeof(udp_eth_ftr_t));
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc =
		udp_crc32_eth(eth_crc, udp->buffer, *size - sizeof(udp_eth_ftr_t));
	if (eth_crc != eth_ftr->crc)
		return NULL;
#endif

	/* extract relevant information */
	memcpy(track->mac, eth_hdr->mac_src, 6);
	memcpy(track->ip, ip_hdr->ip_src, 4);
	track->port = udp_reorder_16(udp_hdr->port_dst);
	parsed = udp->buffer + sizeof(udp_eth_hdr_t) + ihl + sizeof(udp_udp_hdr_t);
	*size = udp_reorder_16(udp_hdr->length) - (uint16_t)(sizeof(udp_udp_hdr_t));

	return parsed;
}

static uint8_t *udp_pack(udp_t *udp, udp_track_t *track, uint16_t *size)
{
	udp_eth_hdr_t *eth_hdr;
#if UDP_ETH_CRC_SEND
	udp_eth_ftr_t *eth_ftr;
#endif
	udp_ip_hdr_t *ip_hdr;
	udp_udp_hdr_t *udp_hdr;
	uint16_t ip_crc;
#if UDP_ETH_CRC_SEND
	uint32_t eth_crc;
#endif

	/* fill ethernet frame header */
	eth_hdr = (udp_eth_hdr_t *)udp->buffer;
	memcpy(eth_hdr->mac_dst, track->mac, 6);
	memcpy(eth_hdr->mac_src, udp->mac, 6);
	eth_hdr->ethertype = udp_reorder_16(UDP_ETH_TYPE_IPV4);

	/* fill IP header */
	ip_hdr = (udp_ip_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t));
	ip_hdr->version_ihl = 0x45;
	ip_hdr->dscp_ecn = 0;
	ip_hdr->length =
		udp_reorder_16(sizeof(udp_ip_hdr_t) + sizeof(udp_udp_hdr_t) + *size);
	ip_hdr->ident = 0;
	ip_hdr->flags_fragment = udp_reorder_16(0x4000);
	ip_hdr->ttl = 0xFF;
	ip_hdr->protocol = UDP_IP_PROTO_UDP;
	memcpy(ip_hdr->ip_dst, track->ip, 4);
	memcpy(ip_hdr->ip_src, udp->ip, 4);
	ip_hdr->crc = 0;
	ip_crc = udp_crc16_ip((const uint8_t *)ip_hdr, sizeof(udp_ip_hdr_t));
	ip_hdr->crc = ip_crc;

	/* fill UDP header */
	udp_hdr = (udp_udp_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t) +
								sizeof(udp_ip_hdr_t));
	udp_hdr->length = udp_reorder_16(*size + sizeof(udp_udp_hdr_t));
	udp_hdr->port_src = 0;
	udp_hdr->port_dst = udp_reorder_16(track->port);
	udp_hdr->crc = 0;

#if UDP_ETH_CRC_SEND
	/* append ethernet checksum */
	eth_ftr =
		(udp_eth_ftr_t *)(udp->buffer + sizeof(udp_eth_hdr_t) +
						  sizeof(udp_ip_hdr_t) + sizeof(udp_udp_hdr_t) + *size);
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc =
		udp_crc32_eth(eth_crc, udp->buffer, *size - sizeof(udp_eth_ftr_t));
	eth_ftr->crc = eth_crc;
#endif

	*size = udp_pack_size(*size);

	return udp->buffer;
}

static int udp_arp(udp_t *udp)
{
	udp_eth_hdr_t *eth_hdr;
	udp_arp_hdr_t *arp_hdr;
	uint8_t src_mac[6];
	uint8_t src_addr[4];
	uint16_t size;
#if UDP_ETH_CRC_SEND
	udp_eth_ftr_t *eth_ftr;
	uint32_t eth_crc;
#endif

	eth_hdr = (udp_eth_hdr_t *)(udp->buffer);
	arp_hdr = (udp_arp_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t));

	/* check if packet is ARP packet */
	if (eth_hdr->ethertype != udp_reorder_16(UDP_ETH_TYPE_ARP))
		return -1;

	/* check if packet's destination MAC is this device */
	if (memcmp(eth_hdr->mac_dst, udp->mac, sizeof(udp->mac)) != 0 &&
		memcmp(eth_hdr->mac_dst, udp_eth_bcast, sizeof(udp->mac)) != 0)
		return -1;

	/* check if ARP IPv4 header is valid */
	if (arp_hdr->htype != udp_reorder_16(1) ||
		arp_hdr->ptype != udp_reorder_16(UDP_ETH_TYPE_IPV4) ||
		arp_hdr->hw_addrlen != 6 || arp_hdr->prot_addrlen != 4 ||
		arp_hdr->oper != udp_reorder_16(UDP_ARP_TYPE_REQUEST) ||
		memcmp(arp_hdr->prot_addr_targ, udp->ip, sizeof(udp->ip)) != 0)
		return -1;

	/* extract source address and mac */
	memcpy(src_mac, arp_hdr->hw_addr_send, sizeof(src_mac));
	memcpy(src_addr, arp_hdr->prot_addr_send, sizeof(src_addr));

	/* fill ethernet frame header */
	memcpy(eth_hdr->mac_dst, src_mac, sizeof(eth_hdr->mac_dst));
	memcpy(eth_hdr->mac_src, udp->mac, sizeof(eth_hdr->mac_src));
	eth_hdr->ethertype = udp_reorder_16(UDP_ETH_TYPE_ARP);

	/* fill ARP frame header */
	arp_hdr->htype = udp_reorder_16(1);
	arp_hdr->ptype = udp_reorder_16(UDP_ETH_TYPE_IPV4);
	arp_hdr->hw_addrlen = 6;
	arp_hdr->prot_addrlen = 4;
	arp_hdr->oper = udp_reorder_16(UDP_ARP_TYPE_REPLY);
	memcpy(arp_hdr->hw_addr_send, udp->mac, sizeof(arp_hdr->hw_addr_send));
	memcpy(arp_hdr->prot_addr_send, udp->ip, sizeof(arp_hdr->prot_addr_send));
	memcpy(arp_hdr->hw_addr_targ, src_mac, sizeof(arp_hdr->hw_addr_targ));
	memcpy(arp_hdr->prot_addr_targ, src_addr, sizeof(arp_hdr->prot_addr_targ));

	size = sizeof(udp_eth_hdr_t) + sizeof(udp_arp_hdr_t);

#if UDP_ETH_CRC_SEND
	/* calculate ethernet frame checksum */
	eth_ftr = (udp_eth_ftr_t *)(udp->buffer + size);
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc = udp_crc32_eth(eth_crc, udp->buffer, size);
	eth_ftr->crc = eth_crc;
	size += sizeof(udp_eth_ftr_t *);
#endif

	udp->phy->send(udp->phy, udp->buffer, size);

	return 0;
}

static int udp_icmp(udp_t *udp, uint16_t size)
{
	udp_eth_hdr_t *eth_hdr;
#if UDP_ETH_CRC_SEND || UDP_ETH_CRC_RECV
	udp_eth_ftr_t *eth_ftr;
#endif
	udp_ip_hdr_t *ip_hdr;
	udp_icmp_hdr_t *icmp_hdr;
	uint16_t ihl;
	uint16_t iplen;
	uint16_t ip_crc;
#if UDP_ETH_CRC_SEND || UDP_ETH_CRC_RECV
	uint32_t eth_crc;
#endif
	uint16_t icmp_crc;

	/* extract ethernet frame header */
	eth_hdr = (udp_eth_hdr_t *)udp->buffer;

	/* check if packet is IPv4 packet */
	if (eth_hdr->ethertype != udp_reorder_16(UDP_ETH_TYPE_IPV4))
		return -1;

	/* check if packet's destination MAC is this device */
	if (memcmp(eth_hdr->mac_dst, udp->mac, sizeof(udp->mac)) != 0 &&
		memcmp(eth_hdr->mac_dst, udp_eth_bcast, sizeof(udp->mac)) != 0)
		return -1;

	/* extract IP header */
	ip_hdr = (udp_ip_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t));

	/* check if IP version is 4 */
	if (ip_hdr->version_ihl >> 4 != 4)
		return -1;

	/* check if protocol is ICMP */
	if (ip_hdr->protocol != UDP_IP_PROTO_ICMP)
		return -1;

	/* check if packet's destination IP is this device */
	if (memcmp(ip_hdr->ip_dst, udp->ip, sizeof(udp->ip)) != 0 &&
		memcmp(ip_hdr->ip_dst, udp_ip_bcast, sizeof(udp->ip) != 0))
		return -1;

	/* extract header length */
	ihl = (ip_hdr->version_ihl & 0x0F) * 4;

	/* extract length */
	iplen = udp_reorder_16(ip_hdr->length);

	/* validate IP header checksum */
	ip_crc = udp_crc16_ip((const uint8_t *)ip_hdr, ihl);
	if (ip_crc != 0)
		return -1;

	/* extract ICMP header */
	icmp_hdr = (udp_icmp_hdr_t *)(udp->buffer + sizeof(udp_eth_hdr_t) + ihl);

	/* check if ICMP message is echo request */
	if (icmp_hdr->type != UDP_ICMP_TYPE_ECHO_REQUEST)
		return -1;

	/* validate ICMP checksum */
	icmp_crc =
		udp_crc16_ip(udp->buffer + sizeof(udp_eth_hdr_t) + ihl, iplen - ihl);
	if (icmp_crc != 0)
		return -1;

#if UDP_ETH_CRC_RECV
	/* validate ethernet frame checksum */
	eth_ftr = (udp_eth_ftr_t *)(udp->buffer + size - sizeof(udp_eth_ftr_t));
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc = udp_crc32_eth(eth_crc, udp->buffer, size - sizeof(udp_eth_ftr_t));
	if (eth_crc != eth_ftr->crc)
		return -1;
#endif

	/* swap MAC and IP addresses for echo reply */
	memcpy(eth_hdr->mac_dst, eth_hdr->mac_src, sizeof(eth_hdr->mac_dst));
	memcpy(eth_hdr->mac_src, udp->mac, sizeof(eth_hdr->mac_src));
	memcpy(ip_hdr->ip_dst, ip_hdr->ip_src, sizeof(ip_hdr->ip_dst));
	memcpy(ip_hdr->ip_src, udp->ip, sizeof(ip_hdr->ip_src));

	/* change ICMP type */
	icmp_hdr->type = UDP_ICMP_TYPE_ECHO_REPLY;

	/* recalculate ICMP checksum */
	icmp_hdr->crc = 0;
	icmp_crc =
		udp_crc16_ip(udp->buffer + sizeof(udp_eth_hdr_t) + ihl, iplen - ihl);
	icmp_hdr->crc = icmp_crc;

	/* recalculate IP checksum */
	ip_hdr->crc = 0;
	ip_crc = udp_crc16_ip((const uint8_t *)ip_hdr, sizeof(udp_ip_hdr_t));
	ip_hdr->crc = ip_crc;

#if UDP_ETH_CRC_SEND
	/* recalculate ethernet frame checksum */
	eth_ftr = (udp_eth_ftr_t *)(udp->buffer + size - sizeof(udp_eth_ftr_t));
	eth_crc = udp_crc32_eth(0, NULL, 0);
	eth_crc = udp_crc32_eth(eth_crc, udp->buffer, size - sizeof(udp_eth_ftr_t));
	eth_ftr->crc = eth_crc;
#endif

#if UDP_ETH_CRC_SEND && !UDP_ETH_CRC_RECV
	size += sizeof(udp_eth_ftr_t);
#endif
#if UDP_ETH_CRC_RECV && !UDP_ETH_CRC_SEND
	size -= sizeof(udp_eth_ftr_t);
#endif

	udp->phy->send(udp->phy, udp->buffer, size);

	return 0;
}

/********************************* End Of File ********************************/