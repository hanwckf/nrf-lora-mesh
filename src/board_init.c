#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf_temp.h"
#include "nrf_drv_saadc.h"
#include "lora_nrf_gpio.h"
#include "compact.h"

const uint8_t leds_list[LEDS_NUMBER] = LED_LIST;

void leds_init(void)
{
	for (uint8_t i=0; i<LEDS_NUMBER; i++) {
		nrf_gpio_cfg_output(leds_list[i]);
		nrf_gpio_pin_write(leds_list[i],0);
	}
}

static void saadc_callback(nrf_drv_saadc_evt_t const * p_event) { }

void saadc_init(void)
{
	ret_code_t err_code;
	nrf_saadc_channel_config_t channel_config =
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VCC_SAMP);
	channel_config.gain = NRF_SAADC_GAIN1_5;
	
	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);
}
