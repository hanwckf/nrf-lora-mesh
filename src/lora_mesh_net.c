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

uint32_t net_tx_ack;
uint32_t net_tx_ack_fail;

QueueHandle_t net_tx_buf;
QueueHandle_t net_rx_buf;

SemaphoreHandle_t m_ack_Semaphore;

static int8_t ra_handle(LoRaPkg* p, lora_net_hook *hook);

TaskHandle_t lora_net_tx_handle;

static uint8_t ack_wait_id = 0;

static int16_t last_seen_pid[255] = { -1 };

#define ACK_TIMEOUT 300
#define ACK_MAX 3

#define NET_TX(p, queue, timeout, h) \
	do { \
		net_tx_cnt++; \
		if (h->netTx != NULL) h->netTx(); \
		xQueueSend(queue, &(p), timeout); \
	} while (0)

#define NET_RX(p, queue, timeout, h) \
	do { \
		net_rx++;\
		if (h->netRecv != NULL) h->netRecv(); \
		xQueueSend(queue, &(p), timeout); \
	} while (0)

#define GEN_RA(p, dest) \
	do { \
		p.Header.type = TYPE_RA; \
		p.Header.MacHeader.dst = MAC_BROADCAST_ADDR; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		p.Header.NetHeader.dst = dest; \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.subtype = SUB_RA; \
	} while (0)

#define GEN_PINGACK(q, p) \
	do { \
		q.Header.type = TYPE_PING; \
		q.Header.MacHeader.dst = p.Header.MacHeader.src; \
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

static int8_t send_wait_ack(LoRaPkg* p, lora_net_hook* hook)
{
	uint8_t tries = 0;
	do {
		tries++;
		xSemaphoreTake(m_ack_Semaphore, 0); /* clean Semaphore */
		NET_TX(p, mac_tx_buf, portMAX_DELAY, hook);
		if (xSemaphoreTake(m_ack_Semaphore, pdMS_TO_TICKS(ACK_TIMEOUT)) == pdPASS)
			return 0; /* ack success */
	} while (tries < ACK_MAX);
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
			/* check broadcast */
			if (p.Header.NetHeader.dst != NET_BROADCAST_ADDR)
			{
				nexthop = Route.getRouteTo(p.Header.NetHeader.dst);
				if (nexthop < 0) {
					/* no nexthop, generate RA, and broadcast it */
					GEN_RA(t, p.Header.NetHeader.dst);
					NET_TX(t, mac_tx_buf, portMAX_DELAY, hook);
				} else {
					/* Route to nexthop */
					p.Header.MacHeader.dst = nexthop;
					if (p.Header.type == TYPE_DATA && p.Header.NetHeader.ack == ACK)
					{
						p.Header.NetHeader.pid = ack_wait_id++;
						if (send_wait_ack(&p, hook) < 0) {
							Route.delRouteByNexthop(nexthop);
							/* ack failed, delete route */
							net_tx_ack_fail++;
						} else {
							net_tx_ack++;
						}
					} else {
						NET_TX(p, mac_tx_buf, portMAX_DELAY, hook);
					}
				}
			} else {
				/* broadcast */
				NET_TX(p, mac_tx_buf, portMAX_DELAY, hook);
			}
		}
	}
}

TaskHandle_t lora_net_rx_handle;

void lora_net_rx_task (void * pvParameter)
{
	mac_net_param_t *param = (mac_net_param_t *)pvParameter;
	lora_net_hook *hook = &(param->net_hooks);
	LoRaPkg p,t;
	
	while (1) {
		if ( xQueueReceive(mac_rx_buf, &p, portMAX_DELAY) == pdPASS)
		{
			if (p.Header.type == TYPE_DATA
				&& p.Header.NetHeader.ack == ACK 
				&& p.Header.MacHeader.dst != MAC_BROADCAST_ADDR
				&& p.Header.NetHeader.dst != NET_BROADCAST_ADDR)
			{
				GEN_ACK(t, p);
				NET_TX(t, mac_tx_buf, portMAX_DELAY, hook);
			}
			
			if (p.Header.NetHeader.dst == Route.getNetAddr()
					|| p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
				switch((uint8_t)p.Header.type) {
					case TYPE_DATA: /* It always be unicast */
						if (p.Header.NetHeader.ack == ACK)
						{
							/* check dup data */
							if (p.Header.NetHeader.pid == last_seen_pid[p.Header.NetHeader.src])
								break;
							else
								last_seen_pid[p.Header.NetHeader.src] = p.Header.NetHeader.pid;
						}
						NET_RX(p, net_rx_buf, 0, hook);
						break;
					case TYPE_DATA_ACK: /* It must be unicast */
						/* check pid */
						if (p.Header.NetHeader.pid == ack_wait_id)
							xSemaphoreGive(m_ack_Semaphore);
						break;
					case TYPE_PING: /* It must be unicast */
						if (p.Header.NetHeader.ack == ACK_NO) {
							/* send pingack (directly send to mac_tx_buf) */
							GEN_PINGACK(t, p);
							NET_TX(t, mac_tx_buf, portMAX_DELAY, hook);
						} else {
							/* It is pingack */
							NET_RX(p, net_rx_buf, 0, hook);
						}
						break;
					case TYPE_RA: /* It can be broadcast(RA) or unicast(RESPON), need to check hops in ra_handle */
						ra_handle(&p, hook);
						break;
					default: net_rx_drop++; break;
				}
			} else {
				/* It must be unicast, check hops */
				if (p.Header.NetHeader.hop > MAX_HOPS) {
					/* hop max, will not forward pkg */
					net_rx_drop++;
				} else {
					if (hook->netForward != NULL) hook->netForward();
					net_fwd++;
					p.Header.NetHeader.hop++;
					NET_TX(p, net_tx_buf, portMAX_DELAY, hook);
				}
			}
		}
	}
}

static int8_t ra_handle(LoRaPkg* p, lora_net_hook *hook)
{
	uint8_t i; LoRaPkg t;
	switch (p->Header.NetHeader.subtype) {
		case SUB_RA:
			/* ignore RA from local */
			if (p->Header.NetHeader.src == Route.getNetAddr())
				return 0;
			
			/* ignore RA already recv */
			for (i=0; i < p->Header.NetHeader.hop; i++) {
				if (p->RouteData.RA_List[i] == Route.getNetAddr())
					return 0;
			}
			
			Route.updateRoute(p->Header.NetHeader.src, 
				p->Header.MacHeader.src, p->Header.NetHeader.hop);
			for (i=0; i < p->Header.NetHeader.hop; i++) {
				Route.updateRoute(p->RouteData.RA_List[i],
					p->Header.MacHeader.src, p->Header.NetHeader.hop - i - 1);
			}
			
			if (p->Header.NetHeader.dst == Route.getNetAddr()) {
				memcpy(&t, p, sizeof(LoRaPkg));
				t.Header.NetHeader.subtype = SUB_RA_RESPON;
				t.Header.NetHeader.dst = p->Header.NetHeader.src;
				t.Header.NetHeader.src = Route.getNetAddr();
				/* send RA_RESPON */
				NET_TX(t, net_tx_buf, portMAX_DELAY, hook);
				return 0;
			}
			
			/* check hops, add NetAddr to RA_List */
			if (p->Header.NetHeader.hop < MAX_HOPS) {
				memcpy(&t, p, sizeof(LoRaPkg));
				t.RouteData.RA_List[t.Header.NetHeader.hop] = Route.getNetAddr();
				t.Header.NetHeader.hop++;
				/* rebroadcast */
				NET_TX(t, net_tx_buf, portMAX_DELAY, hook);
				return 0;
			}
			break;
		case SUB_RA_RESPON: return 0; /* already process in peek_msg */
		case SUB_RA_FAIL: break; /* TODO */
		default: break;
	}
	return -1;
}
