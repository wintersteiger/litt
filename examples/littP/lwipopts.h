#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#include <stdint.h>

#define NO_SYS                      1
#define LWIP_SOCKET                 0

#if PICO_CYW43_ARCH_POLL
#define MEM_LIBC_MALLOC             1
#else
// MEM_LIBC_MALLOC is incompatible with non polling versions
#define MEM_LIBC_MALLOC             0
#endif

#define MEM_ALIGNMENT               4
#define MEM_SIZE                    (128 * 1024)
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETCONN                0
#define MEM_STATS                   1
#define SYS_STATS                   0
#define MEMP_STATS                  1
#define LINK_STATS                  0
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

#ifndef NDEBUG
#define LWIP_DEBUG                  1
#define LWIP_STATS                  1
#define LWIP_STATS_DISPLAY          1
#endif

#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define API_MSG_DEBUG               LWIP_DBG_OFF
#define SOCKETS_DEBUG               LWIP_DBG_OFF
#define ICMP_DEBUG                  LWIP_DBG_OFF
#define INET_DEBUG                  LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define IP_REASS_DEBUG              LWIP_DBG_OFF
#define RAW_DEBUG                   LWIP_DBG_OFF
#define MEM_DEBUG                   LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define TCP_RTO_DEBUG               LWIP_DBG_OFF
#define TCP_CWND_DEBUG              LWIP_DBG_OFF
#define TCP_WND_DEBUG               LWIP_DBG_OFF
#define TCP_FR_DEBUG                LWIP_DBG_OFF
#define TCP_QLEN_DEBUG              LWIP_DBG_OFF
#define TCP_RST_DEBUG               LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define PPP_DEBUG                   LWIP_DBG_OFF
#define SLIP_DEBUG                  LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF

#define LWIP_HTTPD_CGI                1
#define LWIP_HTTPD_SSI                1
#define LWIP_HTTPD_CGI_SSI            1
#define LWIP_HTTPD_FILE_STATE         1
#define LWIP_HTTPD_SSI_RAW            1
#define LWIP_HTTPD_CUSTOM_FILES       1
#define LWIP_HTTPD_DYNAMIC_FILE_READ  1
#define LWIP_HTTPD_FS_ASYNC_READ      1
#define LWIP_HTTPD_FILE_EXTENSION     1
#define LWIP_HTTPD_DYNAMIC_HEADERS    1
#define HTTPD_POLL_INTERVAL           2

#define MEMP_NUM_SYS_TIMEOUT LWIP_NUM_SYS_TIMEOUT_INTERNAL + 5

#define MQTT_REQ_MAX_IN_FLIGHT        16
#define MQTT_REQ_TIMEOUT              1

#define SNTP_SUPPORT                  1
#define SNTP_UPDATE_DELAY             86400000
#define SNTP_MAX_SERVERS              1

#ifdef __cplusplus
extern "C" {
#endif
  extern void sntp_set_system_time_us(uint32_t sec, uint32_t usec);
#ifdef __cplusplus
}
#endif

#define SNTP_SET_SYSTEM_TIME_US(sec, usec) sntp_set_system_time_us(sec, usec)
#define LWIP_SNTP_DEBUG               1

#endif
