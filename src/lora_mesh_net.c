#define DEBUG_LOG
#include "stdlib.h"
#include "sx126x_board.h"
#include "radio.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "lora_mesh.h"
#include "lora_pkg.h"

#include "compact.h"
#include "lora_config.h"

uint32_t net_tx_cnt;
uint32_t net_rx; /* L3 recv count */
uint32_t net_rx_drop; /* L3 drop count */
uint32_t net_fwd; /* routed pkg count */

uint32_t net_tx_ack_ok;
uint32_t net_tx_ack_fail;

QueueHandle_t net_tx_buf;
QueueHandle_t net_rx_buf;

SemaphoreHandle_t m_ack_Semaphore;

uint32_t ack_time;

static bool ra_forward(LoRaPkg* p, lora_net_hook *hook);
static void ra_handle(LoRaPkg* p, lora_net_hook *hook);

TaskHandle_t lora_net_tx_handle;

static uint8_t ack_wait_id = 0;
static uint8_t ra_pid = 0;

static int16_t last_seen_pid[255];
static int16_t last_ra[255];

#define NET_TX(p, queue, timeout, h) \
	do { \
		net_tx_cnt++; \
		if (h->netTx != NULL) h->netTx(); \
		xQueueSend(queue, p, timeout); \
	} while (0)

#define NET_RX(p, queue, timeout, h) \
	do { \
		net_rx++;\
		if (h->netRecv != NULL) h->netRecv(); \
		xQueueSend(queue, p, timeout); \
	} while (0)

#define GEN_RA(p, dest) \
	do { \
		p.Header.type = TYPE_RA; \
		p.Header.MacHeader.dst = MAC_BROADCAST_ADDR; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		p.Header.NetHeader.dst = dest; \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.subtype = SUB_RA; \
		p.Header.NetHeader.pid = ra_pid++; \
	} while (0)

#define GEN_PINGACK(q, p) \
	do { \
		q.Header.type = TYPE_PING; \
		q.Header.NetHeader.src = Route.getNetAddr(); \
		q.Header.NetHeader.dst = p.Header.NetHeader.src; \
		q.Header.NetHeader.hop = 0; \
		q.Header.NetHeader.ack = ACK; \
	} while (0)

#define GEN_ACK(q, p) \
	do { \
		q.Header.type = TYPE_DATA_ACK; \
		q.Header.MacHeader.dst = p.Header.MacHeader.src; \
		q.Header.NetHeader.src = Route.getNetAddr(); \
		q.Header.NetHeader.dst = p.Header.NetHeader.src; \
		q.Header.NetHeader.hop = 0; \
		q.Header.NetHeader.pid = p.Header.NetHeader.pid; \
	} while (0)

#define ACK_TIMEOUT 1000
#define ACK_MAX 3

static int8_t send_wait_ack(LoRaPkg* p, lora_net_hook* hook)
{
	uint8_t tries = 0;
	ack_time = RTOS_TIME;
	do {
		NRF_LOG_DBG_TIME("ack pid: %d, tries: %d", ack_wait_id, tries);
		NET_TX(p, mac_tx_buf, portMAX_DELAY, hook);
		if ( ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ACK_TIMEOUT)) != 0 )
		{
			net_tx_ack_ok++;
			NRF_LOG_DBG_TIME("ack ok!");
			return 0; /* ack success */
		}
		tries++;
	} while (tries < ACK_MAX);
	NRF_LOG_DBG_TIME("ack failed!");
	net_tx_ack_fail++;
	return -1;
}

void lora_net_tx_task (void * pvParameter)
{
	mac_net_param_t *param = (mac_net_param_t *)pvParameter;
	lora_net_hook *hook = &(param->net_hooks);
	LoRaPkg p, t; int8_t nexthop;
	
	while (1) {
		if ( xQueueReceive(net_tx_buf, &p, portMAX_DELAY) == pdPASS)
		{
			/* check dst addr is not local addr */
			if (p.Header.NetHeader.dst == Route.getNetAddr())
				continue;
			NRF_LOG_DBG("==>");
			/* situation 1: NET unicast with MAC broadcast will be forwarded by nodes, eg. RA pkgs 
			   situation 2: NET broadcast with MAC unicast is the same as NET unicast with MAC unicast
			   situation 3: NET broadcast with MAC broadcast and will not be forwarded by any node, 
				 but can be receive by all neighbor nodes.
			   so we set NET broadcast to situation 3 forcedly */

			//NRF_LOG_HEX_DBG(&p, sizeof(LoRaPkg));
			
			if (p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
				p.Header.MacHeader.dst = MAC_BROADCAST_ADDR;
				NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook);
				NRF_LOG_DBG("send net broadcast!");
			} else {
				/* net unicast, findout next hop */
				nexthop = Route.getRouteTo(p.Header.NetHeader.dst);
				if (nexthop < 0) {
					/* no nexthop, generate RA, and broadcast it */
					/* RA with NET unicast and MAC broadcast */
					GEN_RA(t, p.Header.NetHeader.dst);
					NRF_LOG_DBG("No route to dst: 0x%02x, send RA, pid: %d", p.Header.NetHeader.dst, t.Header.NetHeader.pid);
					NET_TX(&t, mac_tx_buf, portMAX_DELAY, hook);
				} else {
					/* Forward to nexthop */
					p.Header.MacHeader.dst = nexthop;
					NRF_LOG_DBG("Tx route dst: 0x%02x, nh: 0x%02x", p.Header.NetHeader.dst, nexthop);
					if (p.Header.type == TYPE_DATA && p.Header.NetHeader.ack == ACK)
					{
						p.Header.NetHeader.pid = ack_wait_id;
						if (send_wait_ack(&p, hook) < 0) {
							Route.delRouteByNexthop(nexthop); /* ack failed, delete route */
							PRINT_ROUTE_TABLE;
						}
						ack_wait_id++;
					} else {
						NRF_LOG_DBG("No need ack, send directly!");
						NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook);
					}
				}
			}
			NRF_LOG_DBG("<==");
		}
	}
}

TaskHandle_t lora_net_rx_handle;

void lora_net_rx_task (void * pvParameter)
{
	mac_net_param_t *param = (mac_net_param_t *)pvParameter;
	lora_net_hook *hook = &(param->net_hooks);
	memset(last_seen_pid, -1, 255);
	memset(last_ra, -1, 255);
	LoRaPkg p,t;
	
	while (1) {
		if ( xQueueReceive(mac_rx_buf, &p, portMAX_DELAY) == pdPASS)
		{
			NRF_LOG_DBG("==>");
			NRF_LOG_DBG("nsrc: 0x%02x, ndst: 0x%02x, msrc: 0x%02x", 
				p.Header.NetHeader.src, p.Header.NetHeader.dst, p.Header.MacHeader.src);
			/*
			if (p.Header.type == TYPE_DATA
				&& p.Header.NetHeader.ack == ACK
				&& p.Header.MacHeader.dst != MAC_BROADCAST_ADDR
				&& p.Header.NetHeader.dst != NET_BROADCAST_ADDR)
			{
				NRF_LOG_DBG_TIME("send ack! pid: %d", p.Header.NetHeader.pid);
				GEN_ACK(t, p);
				NET_TX(&t, mac_tx_buf, portMAX_DELAY, hook);
			}
			*/
			if (p.Header.NetHeader.dst == Route.getNetAddr()
					|| p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
				switch((uint8_t)p.Header.type) {
					case TYPE_DATA: /* It always be unicast */
						NRF_LOG_DBG_TIME("recv: TYPE_DATA");
						if (p.Header.NetHeader.ack == ACK)
						{
							/* check dup data */
							if (p.Header.NetHeader.pid == last_seen_pid[p.Header.NetHeader.src]) {
								NRF_LOG_DBG("dup data!");
								break;
							}
							else {
								last_seen_pid[p.Header.NetHeader.src] = p.Header.NetHeader.pid;
								NRF_LOG_DBG("updata lastseen: %d", p.Header.NetHeader.pid);
							}
						}
						NET_RX(&p, net_rx_buf, 0, hook);
						break;
					case TYPE_DATA_ACK: /* It must be unicast */
						/* check pid */
						NRF_LOG_DBG_TIME("recv: TYPE_DATA_ACK");
						NRF_LOG_DBG("recv pid: %d, wait pid: %d", p.Header.NetHeader.pid, ack_wait_id);
						if (p.Header.NetHeader.pid == ack_wait_id) {
							NRF_LOG_DBG("ack notify!");
							NRF_LOG_DBG("ACK wait: %d", RTOS_TIME - ack_time);
							xTaskNotifyGive(lora_net_tx_handle);
						}
						break;
					case TYPE_PING: /* It must be unicast */
						NRF_LOG_DBG("recv: TYPE_PING");
						if (p.Header.NetHeader.ack == ACK_NO) {
							NRF_LOG_DBG("send ping ack");
							/* send pingack */
							GEN_PINGACK(t, p);
							NET_TX(&t, net_tx_buf, portMAX_DELAY, hook);
						} else {
							NRF_LOG_DBG("recv: ping ack");
							/* It is a pingack */
							NET_RX(&p, net_rx_buf, 0, hook);
						}
						break;
					case TYPE_RA: /* It must be NET unicast (MAC broadcast), need to check hops in ra_handle */
						NRF_LOG_DBG("recv: TYPE_RA");
						ra_handle(&p, hook);
						break;
					default: net_rx_drop++; break;
				}
			} else {
				/* It must be NET unicast, check hops */
				if (p.Header.NetHeader.hop < MAX_HOPS ) {
					NRF_LOG_DBG("forward, now hops: %d", p.Header.NetHeader.hop);
					if (p.Header.type != TYPE_RA || ra_forward(&p, hook)) {
						NRF_LOG_DBG("forward: send to net_tx_buf");
						if (hook->netForward != NULL) hook->netForward();
						net_fwd++;
						p.Header.NetHeader.hop++;
						NET_TX(&p, net_tx_buf, portMAX_DELAY, hook);
					}
				} else {
					NRF_LOG_DBG("hop max, drop!");
					/* hop max, will not forward pkg */
					net_rx_drop++;
				}
			}
			NRF_LOG_DBG("<==");
		}
	}
}

static void send_ra_respon(LoRaPkg* p, lora_net_hook *hook)
{
	NRF_LOG_DBG("recv ra pid: %d, last pid: %d ", p->Header.NetHeader.pid, last_ra[p->Header.NetHeader.src]);
	if (p->Header.NetHeader.pid != last_ra[p->Header.NetHeader.src])
	{
		NRF_LOG_DBG("send RA_RESPON to ndst: 0x%02x!", p->Header.NetHeader.src);
		last_ra[p->Header.NetHeader.src] = p->Header.NetHeader.pid;
		
		p->Header.NetHeader.subtype = SUB_RA_RESPON;
		p->Header.NetHeader.dst = p->Header.NetHeader.src;
		p->Header.NetHeader.src = Route.getNetAddr();
		
		//NRF_LOG_HEX_DBG(p, sizeof(LoRaPkg));
		
		NET_TX(p, net_tx_buf, portMAX_DELAY, hook);
	}
}

static void ra_handle(LoRaPkg* p, lora_net_hook *hook)
{
	switch (p->Header.NetHeader.subtype) {
		case SUB_RA: send_ra_respon(p, hook);
			break;
		case SUB_RA_RESPON: break; /* already process in mac_peek_pkg */
		case SUB_RA_FAIL: break; /* TODO */
		default: break;
	}
}

/* return true: NEED to forward */
/* return false: already enqueue to mac_tx_buf or ignored */

static bool ra_forward(LoRaPkg* p, lora_net_hook *hook)
{
	uint8_t i;
	switch (p->Header.NetHeader.subtype) {
		case SUB_RA:
			/* ignore RA from local */
			if (p->Header.NetHeader.src == Route.getNetAddr())
				return false;

			/* ignore RA already recv */
			for (i=0; i < p->Header.NetHeader.hop; i++) {
				if (p->RouteData.RA_List[i] == Route.getNetAddr())
					return false;
			}

			/* add NetAddr to RA_List, then rebroadcast */
			p->RouteData.RA_List[p->Header.NetHeader.hop] = Route.getNetAddr();
			p->Header.NetHeader.hop++;
			NRF_LOG_DBG("ra: rebroadcast!");
			for (i=0; i < p->Header.NetHeader.hop; i++) {
				NRF_LOG_DBG("RA_List_%d: 0x%02x", i , p->RouteData.RA_List[i]);
			}
			NET_TX(p, mac_tx_buf, portMAX_DELAY, hook);
			return false;
		case SUB_RA_RESPON: break; /* already process in peek_msg */
		case SUB_RA_FAIL: break; /* TODO */
		default: break;
	}
	return true;
}
