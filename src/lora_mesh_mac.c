//#define DEBUG_LOG
//#define DEBUG_ROUTE

#include "stdlib.h"
#include "sx126x_board.h"
#include "radio.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "lora_mesh.h"
#include "lora_pkg.h"

#include "nrf_log_ctrl.h"

#include "lora_config.h"
#include "mesh_route.h"

/* phy layer counter */
uint32_t phy_cad_done;
uint32_t phy_cad_det;
uint32_t phy_rx_done;
uint32_t phy_rx_err;
uint32_t phy_rx_timeout;
uint32_t phy_tx_done;
uint32_t phy_tx_err;

/* mac layer counter */
uint32_t mac_rx_done;
uint32_t mac_rx_drop;
uint32_t mac_tx_done;
uint32_t mac_ack_respon;

QueueHandle_t mac_tx_buf;
QueueHandle_t mac_rx_buf;

static uint8_t tx_timer;

SemaphoreHandle_t m_irq_Semaphore;

#define GEN_ACK(q, p) \
	do { \
		q.Header.type = TYPE_DATA_ACK; \
		q.Header.MacHeader.dst = p->Header.MacHeader.src; \
		q.Header.NetHeader.src = Route.getNetAddr(); \
		q.Header.NetHeader.dst = p->Header.MacHeader.src; \
		q.Header.NetHeader.hop = 0; \
		q.Header.NetHeader.pid = p->Header.NetHeader.pid; \
	} while (0)

void mac_peek_pkg (LoRaPkg* p)
{
	uint8_t i,j;
	/* update link quality hashmap */
	Route.updateLinkQualityMap(p->Header.MacHeader.src, p->stat.RssiPkt);

	PRINT_LINKQUALITY_MAP;

	Route.updateRoute(p->Header.NetHeader.src,
		p->Header.MacHeader.src, p->Header.NetHeader.hop);

	if (p->Header.type == TYPE_RA)
	{
		if ( p->Header.NetHeader.subtype == SUB_RA ) {
			NRF_LOG_DBG_TIME("L2: RA peek");
		} else if ( p->Header.NetHeader.subtype == SUB_RA_RESPON ) {
			NRF_LOG_DBG_TIME("L2: RA_RESPON peek");
		}
		
		NRF_LOG_DBG_TIME("L2: nsrc: 0x%02x, ndst: 0x%02x, msrc: 0x%02x",
			p->Header.NetHeader.src, p->Header.NetHeader.dst, p->Header.MacHeader.src);
		for (i=0; i < p->Header.NetHeader.hop; i++) {
			NRF_LOG_DBG("L2: RA_List_%d: 0x%02x", i , p->RouteData.RA_List[i]);
		}

		if (p->Header.NetHeader.subtype == SUB_RA)
		{
			/* ignore RA from local */
			if (p->Header.NetHeader.src == Route.getNetAddr())
				return;
			
			/* ignore previous received RA */
			for (i=0; i < p->Header.NetHeader.hop; i++) {
				if (p->RouteData.RA_List[i] == Route.getNetAddr())
					return;
			}

			for (i=0; i < p->Header.NetHeader.hop; i++) {
				Route.updateRoute(p->RouteData.RA_List[i],
					p->Header.MacHeader.src, p->Header.NetHeader.hop - i - 1);
			}
		} else if (p->Header.NetHeader.subtype == SUB_RA_RESPON)
		{
			for (i = 0; i < p->Header.NetHeader.hop; i++) {
				if (p->RouteData.RA_List[i] == Route.getMacAddr())
					break;
			}
			i++;
			for (j=0 ; i < p->Header.NetHeader.hop; i++, j++) {
				Route.updateRoute(p->RouteData.RA_List[i],
					p->Header.MacHeader.src, j);
			}
		}
	}
}

extern uint8_t ack_wait_id;
extern uint32_t ack_time;

static void mac_rx_handle(LoRaPkg* p)
{
	uint8_t mac_dst = p->Header.MacHeader.dst;
	mac_peek_pkg(p);
	LoRaPkg t;
	if (mac_dst == Route.getMacAddr() || mac_dst == MAC_BROADCAST_ADDR) {
		mac_rx_done++;
		if (p->Header.type == TYPE_DATA 
				&& p->Header.NetHeader.ack == ACK
				&& mac_dst != MAC_BROADCAST_ADDR 
				&& p->Header.NetHeader.dst != NET_BROADCAST_ADDR) {
			mac_ack_respon++;
			NRF_LOG_DBG_TIME("L2: send ack! pid: %d", p->Header.NetHeader.pid);
			GEN_ACK(t, p);
			xQueueSend(mac_tx_buf, &t, 0);

			/* check dup data */
			if (p->Header.NetHeader.pid == _last_seen_pid[p->Header.MacHeader.src]) {
				NRF_LOG_DBG_TIME("L2: dup data, drop!");
				return;
			} else {
				_last_seen_pid[p->Header.MacHeader.src] = p->Header.NetHeader.pid;
				NRF_LOG_DBG_TIME("L2: update last pid from 0x%02x: %d", p->Header.MacHeader.src, p->Header.NetHeader.pid);
			}
		} else if (p->Header.type == TYPE_DATA_ACK) {
			NRF_LOG_DBG_TIME("L2: recv TYPE_DATA_ACK pid: %d, wait pid: %d", p->Header.NetHeader.pid, ack_wait_id);
			if (p->Header.NetHeader.pid == ack_wait_id) {
				NRF_LOG_DBG_TIME("L2: ack notify, Delay: %d", RTOS_TIME - ack_time);
				xTaskNotifyGive(lora_net_tx_handle);
			} else {
				NRF_LOG_DBG_TIME("L2: ack ignore!");
			}
			return;
		}
		xQueueSend(mac_rx_buf, p, 0);
	} else {
		mac_rx_drop++;
	}
}

#define SET_RADIO(fun,irq) \
	do { BoardEnableIrq(); \
		fun; \
		xSemaphoreTake(m_irq_Semaphore, portMAX_DELAY); \
		irq = SX126xGetIrqStatus( ); \
		SX126xClearIrqStatus( IRQ_RADIO_ALL ); \
	} while (0)

#define IS_IRQ(irq,x) \
	(((irq) & (x) ) == x)

TaskHandle_t lora_mac_handle;

void lora_mac_task(void * pvParameter)
{
	uint16_t irqRegs; uint32_t timer;
	UNUSED_VARIABLE(timer);
	uint8_t pkgsize;
	uint8_t pkgbuf[255];
	PkgType hdr_type;
	LoRaPkg rxtmp, txtmp;
	PacketStatus_t RadioPktStatus;
	mac_net_param_t *param = (mac_net_param_t * )pvParameter;
	lora_mac_hook *hook = &(param->mac_hooks);
	
	while (1) {
		NRF_LOG_FLUSH();
		Radio.Standby();
		SET_RADIO( Radio.StartCad(), irqRegs );
		if (IS_IRQ(irqRegs, IRQ_CAD_DONE)) 
		{
			phy_cad_done++;			
			if (hook->macCadDone != NULL) hook->macCadDone();
			
            if (IS_IRQ(irqRegs, IRQ_CAD_ACTIVITY_DETECTED)) 
			{
				phy_cad_det++;
				if (hook->macCadDetect != NULL) hook->macCadDetect();
				/* cad detected, set RX */
				Radio.SetMaxPayloadLength( MODEM_LORA, 0xff );
				
				//NRF_LOG_DBG_TIME("L1: Rx start");
				timer = RTOS_TIME;
				if (hook->macRxStart != NULL) hook->macRxStart();
				SET_RADIO( Radio.Rx(RX_TIMEOUT), irqRegs );
				if (hook->macRxEnd != NULL) hook->macRxEnd();

				if ((IS_IRQ(irqRegs, IRQ_CRC_ERROR)) || (IS_IRQ(irqRegs, IRQ_HEADER_ERROR))) {
					phy_rx_err++;
					NRF_LOG_DBG_TIME("L1: Rx error!");
				} else if (IS_IRQ(irqRegs, IRQ_RX_TX_TIMEOUT)) {
					phy_rx_timeout++;
					NRF_LOG_DBG_TIME("L1: Rx timeout!");
				} else if (IS_IRQ(irqRegs, IRQ_RX_DONE)) {
					phy_rx_done++;
					SX126xGetPayload(pkgbuf, &pkgsize, 255);
					NRF_LOG_DBG_TIME("L1: Rx done, size: %d, time: %d", pkgsize, RTOS_TIME - timer);

					hdr_type = (PkgType) (pkgbuf[0]);
					//NRF_LOG_DBG_TIME("L1: Rx hex:");
					//NRF_LOG_HEX_DBG(pkgbuf, pkgsize);
					if (hdr_type < TYPE_MAX && pkgsize == pkgSizeMap[hdr_type][1])
					{
						memcpy(&rxtmp, pkgbuf, pkgsize);
						SX126xGetPacketStatus( &RadioPktStatus );
						rxtmp.stat.RssiPkt = RadioPktStatus.Params.LoRa.RssiPkt;
						rxtmp.stat.SignalRssiPkt = RadioPktStatus.Params.LoRa.SignalRssiPkt;
						rxtmp.stat.SnrPkt = RadioPktStatus.Params.LoRa.SnrPkt;
						
						mac_rx_handle(&rxtmp);
					}
				} else {
					phy_rx_err++;
					NRF_LOG_DBG_TIME("L1: Rx unknown error!");
				}
			} else
			{
				/* cad not detected, check whether phy_tx_buf is empty, then check tx timer */
				if ( uxQueueMessagesWaiting(mac_tx_buf) != 0 ) {
					/* phy_tx_buf is not empty */
					/* check whether tx timer timeout */
					if ( tx_timer == 0 && xQueueReceive(mac_tx_buf, &txtmp, 0) == pdPASS ) {
						mac_tx_done++;
						/* tx timer timeout, reset timer counter */
						tx_timer = (xTaskGetTickCount() & TX_TIMER_MASK );

						/* determin the actual size to send */
						hdr_type = txtmp.Header.type;
						if (hdr_type < TYPE_MAX) {
							pkgsize = pkgSizeMap[hdr_type][1];
						} else {
							pkgsize = SIZE_PKG_MAX;
						}

						txtmp.Header.MacHeader.src = Route.getMacAddr();

						//NRF_LOG_DBG_TIME("L1: Tx start!");
						//NRF_LOG_DBG_TIME("L1: Tx hex:");
						//NRF_LOG_HEX_DBG(&txtmp, pkgsize);
						timer = RTOS_TIME;
						if (hook->macTxStart != NULL) hook->macTxStart();
						SET_RADIO(Radio.Send((uint8_t *)&txtmp, pkgsize, TX_TIMEOUT), irqRegs);
						if (hook->macTxEnd != NULL) hook->macTxEnd();
						
						if (txtmp.Header.type == TYPE_DATA && txtmp.Header.NetHeader.ack == ACK)
							xSemaphoreGive(m_ack_Semaphore);
						
						if (IS_IRQ(irqRegs, IRQ_TX_DONE)) {
							NRF_LOG_DBG_TIME("L1: Tx done, size: %d, time: %d", pkgsize, RTOS_TIME - timer);
							phy_tx_done++;
						} else {
							NRF_LOG_DBG_TIME("L1: Tx error/timeout!");
							phy_tx_err++;
						}
					} else {
						//NRF_LOG_DBG_TIME("L1: wait Tx timer: %d...", tx_timer);
						--tx_timer;
					}
				}
			}
        }
		
		Radio.Sleep();
		NRF_LOG_FLUSH();
		vTaskDelay(pdMS_TO_TICKS(CAD_PERIOD_MS));
	}
}
