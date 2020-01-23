#ifndef __LORA_MESH_
#define __LORA_MESH_

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#define MAC_BROADCAST_ADDR 0xff
#define NET_BROADCAST_ADDR MAC_BROADCAST_ADDR

extern TaskHandle_t lora_net_tx_handle;

void lora_net_tx_task (void * pvParameter);

extern TaskHandle_t lora_net_rx_handle;

void lora_net_rx_task (void * pvParameter);

extern TaskHandle_t lora_mac_handle;

void lora_mac_task(void * pvParameter);

extern QueueHandle_t mac_tx_buf;
extern QueueHandle_t mac_rx_buf;
extern QueueHandle_t net_tx_buf;
extern QueueHandle_t net_rx_buf;

extern SemaphoreHandle_t m_irq_Semaphore;

extern uint32_t mac_cad_done;
extern uint32_t mac_cad_det;
extern uint32_t mac_rx_done;
extern uint32_t mac_rx_err;
extern uint32_t mac_rx_timeout;
extern uint32_t mac_rx_drop; /* L2 drop count */
extern uint32_t mac_tx_done;
extern uint32_t mac_tx_err;
extern uint32_t mac_rx_peek;

extern uint32_t net_tx_cnt;
extern uint32_t net_rx; /* L3 rx count */
extern uint32_t net_rx_drop; /* L3 drop count */
extern uint32_t net_fwd; /* routed pkg count */
extern uint32_t net_tx_ack_ok;
extern uint32_t net_tx_ack_fail;

typedef struct {
	void (* macRxStart) (void);
	void (* macTxStart) (void);
	void (* macRxEnd) (void);
	void (* macTxEnd) (void);
	void (* macCadDone) (void);
	void (* macCadDetect) (void);
} lora_mac_hook;

typedef struct {
	void (* netRecv) (void);
	void (* netForward) (void);
	void (* netTx) (void);
} lora_net_hook;

typedef struct {
	lora_net_hook net_hooks;
	lora_mac_hook mac_hooks;
} mac_net_param_t;

#endif
