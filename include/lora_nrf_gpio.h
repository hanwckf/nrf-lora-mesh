#ifndef LORA_NRF_GPIO__
#define LORA_NRF_GPIO__

/* LEDs */
#define LEDS_NUMBER		4

#define LED_RED			7
#define LED_GREEN		8
#define LED_YELLOW		11
#define LED_BLUE		12

#define LED_LIST { LED_RED, LED_GREEN, LED_YELLOW, LED_BLUE }

/* SX126x */
#define SX126x_DIO1		25
#define SX126x_BUSY		26
#define SX126x_NRST		27
#define SX126x_MISO		28
#define SX126x_MOSI		29
#define SX126x_SCLK		30
#define SX126x_NSS		31
#define SX126x_RXEN		2

/* optional gpio */
#define OPT_GPIO_1		6
#define OPT_GPIO_2		4
#define OPT_GPIO_3		3
#define OPT_GPIO_4		22

/* others */
#define VCC_SAMP		NRF_SAADC_INPUT_AIN3
#define UART_RXD		13
#define UART_TXD		14

#endif
