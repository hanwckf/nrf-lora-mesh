#ifndef __COMPACT_DRV__
#define __COMPACT_DRV__

#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"

#define DelayMs(i) nrf_delay_ms(i)
#define Spi_t	nrf_drv_spi_t

#define NRF_LOG(fmt, ...) NRF_LOG_RAW_INFO("%s: " fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#define NRF_LOG_HEX NRF_LOG_RAW_HEXDUMP_INFO

#ifdef DEBUG_LOG
#define NRF_LOG_DBG(fmt, ...) NRF_LOG_RAW_INFO("DBG: %s: " fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#define NRF_LOG_HEX_DBG NRF_LOG_RAW_HEXDUMP_INFO
#else
#define NRF_LOG_DBG(fmt, ...)
#define NRF_LOG_HEX_DBG(p_data, len)
#endif

#endif
