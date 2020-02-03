#define DEBUG_LOG
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_temp.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_clock.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "radio.h"
#include "sx126x_board.h"

#include "lora_mesh.h"
#include "lora_pkg.h"

#include "lora_config.h"
#include "mesh_route.h"

#include "lora_nrf_gpio.h"
#include "board_init.h"

void dio_irq_handle ( nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action )
{
	BaseType_t yield_req = pdFALSE;
	BoardDisableIrq( );
	xSemaphoreGiveFromISR(m_irq_Semaphore, &yield_req);
	portYIELD_FROM_ISR(yield_req);
}

#define ACK_CONFIG ACK

#define APP_TX(p, queue, timeout) \
	do { \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		xQueueSend(queue, &p, 0); \
	} while (0)

#define UPLOAD_PERIOD_MS		5000

#define VOLT_DIV				2
#define ADC_FULL_SCALE			3
#define ADC_TO_mV(x) \
	((int) ((1000 * (x) * ADC_FULL_SCALE ) >> 10))

#define DST						0x1

static TaskHandle_t app_upload_handle;
static void app_upload_task (void * pvParameter)
{
	float volatile temp;
	nrf_saadc_value_t volt;
	LoRaPkg p = {
		.Header.type = TYPE_DATA,
		.Header.NetHeader.dst = DST,
		.Header.NetHeader.subtype = SUB_DATA,
		.Header.NetHeader.ack = ACK_CONFIG,
	};

	while (1)
	{
		NRF_TEMP->TASKS_START = 1; /* Start the temperature measurement. */
		while (NRF_TEMP->EVENTS_DATARDY == 0) { }
		NRF_TEMP->EVENTS_DATARDY = 0;
		temp = ((float)nrf_temp_read()) / 4;
		NRF_TEMP->TASKS_STOP = 1;
		nrf_drv_saadc_sample_convert(0, &volt);
		NRF_LOG_TIME("Temp: %d, Vcc: %d mV", (int)temp, ADC_TO_mV(volt) * VOLT_DIV);

		p.AppData.temp = temp;
		p.AppData.volt = (uint16_t) (ADC_TO_mV(volt) * VOLT_DIV);

		APP_TX(p, net_tx_buf, 0);
		vTaskDelay(pdMS_TO_TICKS(UPLOAD_PERIOD_MS));
	}
}

#define PING_TIMEOUT 3000

typedef struct {
	uint8_t upload_node;
	uint8_t ping_dest;
} Ping_t;

static TaskHandle_t app_ping_handle;
static QueueHandle_t app_ping_buf;
static int8_t ping_dst = -1;

static void app_ping_task (void * pvParameter)
{
	Ping_t ping_req; uint32_t ret;
	int16_t time;
	LoRaPkg p = {
		.Header.type = TYPE_PING,
		.Header.NetHeader.ack = ACK_NO,
	};
	LoRaPkg u = {
		.Header.type = TYPE_DATA,
		.Header.NetHeader.ack = ACK_CONFIG,
		.Header.NetHeader.subtype = SUB_CONTROL,
		.AppData.custom[0] = CMD_UPLOAD_PING,
	};
	
	while (1) {
		if ( xQueueReceive(app_ping_buf, &ping_req, portMAX_DELAY) == pdTRUE )
		{
			p.Header.NetHeader.dst = ping_dst = ping_req.ping_dest;
			
			time = (xTaskGetTickCount() * 1000) >> 10 ;
			APP_TX(p, net_tx_buf, 0);
			
			ret = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PING_TIMEOUT));
			
			if (ret != 0) {
				time = (uint16_t) (((xTaskGetTickCount() * 1000) >> 10) - time);
			} else {
				time = -1;
			}
			/* upload ret */
			if (Route.getNetAddr() == ping_req.upload_node) {
				if (time < 0) {
					NRF_LOG_TIME("0x%02x -> 0x%02x, timeout!",Route.getNetAddr(), ping_req.ping_dest);
				} else {
					NRF_LOG_TIME("0x%02x -> 0x%02x, %d ms",Route.getNetAddr(), ping_req.ping_dest, time);
				}
			} else {
				u.Header.NetHeader.dst = ping_req.upload_node;
				u.AppData.custom[0] = CMD_UPLOAD_PING;
				u.AppData.custom[1] = ping_req.ping_dest;
				memcpy(u.AppData.custom + 2, &time, sizeof(time));
				APP_TX(u, net_tx_buf, 0);
			}
		}
	}
}

static void print_data (LoRaPkg *p)
{
	NRF_LOG_TIME("===>");
	NRF_LOG("NET: 0x%02x, MAC: 0x%02x(%d), Rssi: %d, Snr: %d", p->Header.NetHeader.src, p->Header.MacHeader.src,
		p->Header.NetHeader.hop, p->stat.RssiPkt, p->stat.SnrPkt);
	NRF_LOG_RAW_INFO("__upd__[%d][" NRF_LOG_FLOAT_MARKER "][%d]\n",p->Header.NetHeader.src, NRF_LOG_FLOAT(p->AppData.temp),p->AppData.volt);
	NRF_LOG_TIME("<===");
}

static void print_pingret (LoRaPkg *p)
{
	int16_t ping_ret;
	memcpy(p->AppData.custom + 2, &ping_ret, sizeof(ping_ret));
	if (ping_ret < 0) {
		NRF_LOG_TIME("0x%02x -> 0x%02x, timeout!", p->Header.NetHeader.src, p->AppData.custom[1]);
	} else {
		NRF_LOG_TIME("0x%02x -> 0x%02x, %d ms", p->Header.NetHeader.src, p->AppData.custom[1], ping_ret);
	}
}

static void notify_ping_app (LoRaPkg *p)
{
	Ping_t ping_req;
	ping_req.ping_dest = p->AppData.custom[1];
	ping_req.upload_node = p->Header.NetHeader.src;
	xQueueSend(app_ping_buf, &ping_req, 0);
}

static void led_control (LoRaPkg *p)
{
	for (uint8_t i=0; i<LEDS_NUMBER; i++) {
		nrf_gpio_cfg_output(leds_list[i]);
		nrf_gpio_pin_write(leds_list[i], p->AppData.custom[i+1]);
	}
}

static void app_control (LoRaPkg *p)
{
	uint8_t subtype = (uint8_t)p->Header.NetHeader.subtype;

	switch (subtype) {
		case SUB_DATA:
			print_data(p);
			break;
		case SUB_CONTROL:
			switch (p->AppData.custom[0]) {
				case CMD_PING: 
					notify_ping_app(p);
					break;
				case CMD_UPLOAD_PING: print_pingret(p);
					break;
				case CMD_LED: led_control(p);
					break;
				case CMD_GET_STAT: /* TODO */
					break;
				case CMD_UPLOAD_STAT: /* TODO */
					break;
				default: break;
			}
			break;
		default:
			break;
	}
}

static TaskHandle_t app_recv_handle;
static void app_recv_task (void * pvParameter)
{
	LoRaPkg p; uint8_t type;

	while (1) {
		if ( xQueueReceive(net_rx_buf, &p, portMAX_DELAY) == pdPASS )
		{
			type = (uint8_t)p.Header.type;
			switch (type) {
				case TYPE_DATA: app_control(&p);
					break;
				case TYPE_PING: /* ping ack, without payload */
					if (ping_dst == p.Header.NetHeader.src)
						xTaskNotifyGive(app_ping_task);
					break;
				default: break;
			}
		}
	}
}

static TaskHandle_t app_gw_handle;
static void app_gw_task (void * pvParameter)
{
	Ping_t ping_req;
	ping_req.upload_node = *(uint8_t *)pvParameter;
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		ping_req.ping_dest = 0x2;
		xQueueSend(app_ping_buf, &ping_req, 0);
		
		vTaskDelay(pdMS_TO_TICKS(5000));
		ping_req.ping_dest = 0x3;
		xQueueSend(app_ping_buf, &ping_req, 0);	
	}
}

#define STAT_PERIOD_MS			10000

#define PRINT_STAT_LOCAL
static TaskHandle_t app_stat_handle;
static QueueHandle_t app_stat_buf;
static void app_stat_task ( void * pvParameter)
{
	uint8_t dst; LoRaPkg t = {
		.Header.type = TYPE_DATA,
		.Header.NetHeader.subtype = SUB_CONTROL,
		.Header.NetHeader.ack = ACK_CONFIG,
		.AppData.custom[0] = CMD_UPLOAD_STAT,
	};
	
	while (1) {
		if ( xQueueReceive(app_stat_buf, &dst, pdMS_TO_TICKS(STAT_PERIOD_MS)) == pdTRUE ) {
			NRF_LOG_TIME("L7: CMD_UPLOAD_STAT recv!")
			t.Header.NetHeader.dst = dst;
			memcpy(t.AppData.custom + 1 , &mac_tx_done, sizeof(mac_tx_done));
			APP_TX(t, net_tx_buf, 0);
		}
#ifdef PRINT_STAT_LOCAL
		NRF_LOG_TIME("===>");
		NRF_LOG("PHY CAD det/done: %d, %d", phy_cad_det, phy_cad_done);
		NRF_LOG("PHY Rx err/timeout/done: %d, %d, %d", phy_rx_err, phy_rx_timeout, phy_rx_done);
		NRF_LOG("MAC Tx: %d; Rx: %d", mac_tx_done, mac_rx_done);
		NRF_LOG("NET Rx: %d; Tx acked: %d (retry: %d, fail: %d)", net_rx_done, net_tx_ack_ok, net_tx_ack_retry, net_tx_ack_fail);
		NRF_LOG_TIME("<===");
#endif
	}
}

void mac_tx_start_hook (void) {
	nrf_gpio_pin_set(LED_YELLOW);
}

void mac_rx_start_hook (void) {
	nrf_gpio_pin_set(LED_GREEN);
}

void mac_tx_end_hook (void) {
	nrf_gpio_pin_clear(LED_YELLOW);
}

void mac_rx_end_hook (void) {
	nrf_gpio_pin_clear(LED_GREEN);
}

void mac_cad_done_hook (void) {
	nrf_gpio_pin_toggle(LED_RED);
}

#define NET_TX_BUF_NUM 8
#define NET_RX_BUF_NUM 8
#define MAC_TX_BUF_NUM 8
#define MAC_RX_BUF_NUM 8

int main(void)
{
	TimerHandle_t LinkQMap_timer;
	RadioStatus_t status;
	RadioError_t err;

	NRF_POWER->DCDCEN = 1;
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	APP_ERROR_CHECK(nrf_drv_clock_init());

#if defined (__ARMCC_VERSION)
	NRF_LOG("ARMCC version: %d", __ARMCC_VERSION);
#elif defined (__VERSION__)
	NRF_LOG("GCC version: %s", __VERSION__);
#endif
	NRF_LOG("Compile Time: %s %s", __DATE__, __TIME__);

	leds_init();
	nrf_temp_init();
	saadc_init();

	SX126xIoInit(SX126x_NSS, SX126x_MISO, SX126x_MOSI, SX126x_SCLK, 
		SX126x_BUSY, SX126x_RXEN, SX126x_NRST, SX126x_DIO1);
	
    Radio.Init( NULL , dio_irq_handle);
    Radio.SetChannel( RF_FREQ );

    Radio.SetTxConfig( MODEM_LORA, TX_POWER, 0, LORA_BW, LORA_SF, LORA_CR,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 0 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BW, LORA_SF, LORA_CR, 0, LORA_PREAMBLE_LENGTH,
                                   0, LORA_FIX_LENGTH_PAYLOAD_ON, 
								   0, true, 0, 0, LORA_IQ_INVERSION_ON, false );

	SX126xConfigureCad(CAD_SYMBOL_NUM, CAD_DET_PEAK, CAD_DET_MIN, 0);
	
	status = SX126xGetStatus();
	if (status.Fields.ChipMode == 0x2) {
		NRF_LOG("sx126x status: STDBY_RC");
	} else {
		NRF_LOG("sx126x mode: 0x%02x", status.Fields.ChipMode);
	}
	
	err = SX126xGetDeviceErrors();
	if (!err.Value) {
		NRF_LOG("sx126x init with 0 error");
	} else {
		NRF_LOG("sx126x err: 0x%02x", err.Value);
	}
	
	static mac_net_param_t param = {
		.mac_hooks.macCadDone = mac_cad_done_hook,
		.mac_hooks.macRxStart = mac_rx_start_hook,
		.mac_hooks.macTxStart = mac_tx_start_hook,
		.mac_hooks.macRxEnd = mac_rx_end_hook,
		.mac_hooks.macTxEnd = mac_tx_end_hook,
	};

#ifdef ENABLE_SEGGER_SYSTEMVIEW
	SEGGER_SYSVIEW_Conf();
#endif

	net_tx_buf = xQueueCreate(NET_TX_BUF_NUM, sizeof(LoRaPkg));
	net_rx_buf = xQueueCreate(NET_RX_BUF_NUM, sizeof(LoRaPkg));
	mac_tx_buf = xQueueCreate(MAC_TX_BUF_NUM, sizeof(LoRaPkg));
	mac_rx_buf = xQueueCreate(MAC_RX_BUF_NUM, sizeof(LoRaPkg));
	app_ping_buf = xQueueCreate(8, sizeof(Ping_t));
	app_stat_buf = xQueueCreate(8, sizeof(uint8_t));
	
	m_irq_Semaphore = xSemaphoreCreateBinary();
	m_ack_Semaphore = xSemaphoreCreateBinary();

	NRF_LOG("LoRaPkg max size: %d", SIZE_PKG_MAX);
	Addr_t addr;

	do {
		if ( NRF_UICR->CUSTOMER[0] == 0xdeadbeef ) {
			addr.mac = addr.net = NRF_UICR->CUSTOMER[1];
			Route.initRouteTable(&addr);
			NRF_LOG("Using UICR as addr: 0x%02x", NRF_UICR->CUSTOMER[1]);
		} else {
			NRF_LOG("ERROR! Invalid UICR register");
			break;
		}

		if ( net_tx_buf && net_rx_buf && mac_tx_buf && mac_rx_buf && app_ping_buf && app_stat_buf && m_irq_Semaphore && m_ack_Semaphore ) {
			xTaskCreate(lora_mac_task, "lora_mac", configMINIMAL_STACK_SIZE + 200, &param, 3, &lora_mac_handle);
			
			xTaskCreate(lora_net_tx_task, "lora_net_tx", configMINIMAL_STACK_SIZE + 200, &param, 2, &lora_net_tx_handle);
			xTaskCreate(lora_net_rx_task, "lora_net_rx", configMINIMAL_STACK_SIZE + 200, &param, 2, &lora_net_rx_handle);
			if (NRF_UICR->CUSTOMER[1] == 0x1) {
				/* ping to id 0x2 and 0x3 periodly */
				//xTaskCreate(app_gw_task, "app_gw", configMINIMAL_STACK_SIZE + 100, NULL, 1, &app_gw_handle);
			} else {
				xTaskCreate(app_upload_task, "app_upload", configMINIMAL_STACK_SIZE + 100, NULL, 1, &app_upload_handle);
			}
			xTaskCreate(app_recv_task, "app_recv", configMINIMAL_STACK_SIZE + 200, NULL, 1, &app_recv_handle);
			xTaskCreate(app_stat_task, "app_stat", configMINIMAL_STACK_SIZE + 100, NULL, 1, &app_stat_handle);
			xTaskCreate(app_ping_task, "app_ping", configMINIMAL_STACK_SIZE + 100, NULL, 1, &app_ping_handle);

			LinkQMap_timer = xTimerCreate("LQM_timer", pdMS_TO_TICKS(LINKQMAP_CLEAR_PERIOD), pdTRUE, 0, Route.clearLinkQuailtyMapTimer);
			xTimerStart(LinkQMap_timer, 0);
			
			NRF_LOG("nrf lora mesh starting...");
			NRF_LOG_FLUSH();
			
			SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
			
			vTaskStartScheduler();
		
		} else {
			NRF_LOG("ERROR! xQueueCreate() or xSemaphoreCreateBinary() failed!");
			break;
		}
	} while (0);

	NRF_LOG_FLUSH();
	
	while(1);

}
