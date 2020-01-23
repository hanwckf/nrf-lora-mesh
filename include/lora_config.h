#ifndef __LORA_CONF_
#define __LORA_CONF_

#define RF_FREQ							434000000UL /* 434MHz */
#define TX_POWER						22  /* see SX126xSetTxParams() */

#define LORA_BW							0	/* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz,3: Reserved] */
#define LORA_SF							7   /* [SF7..SF12] */
#define LORA_CR							1   /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define LORA_PREAMBLE_LENGTH			60
#define LORA_FIX_LENGTH_PAYLOAD_ON		false
#define LORA_IQ_INVERSION_ON			false

//CAD parameters
#define CAD_SYMBOL_NUM                  LORA_CAD_02_SYMBOL
#define CAD_DET_PEAK                    22
#define CAD_DET_MIN                     10

#define CAD_PERIOD_MS                   30
#define RX_TIMEOUT                      100
#define TX_TIMEOUT                      250
#define TX_TIMER_MASK                   0x3 /* tx timer is between (0 ~ 3) * CAD_PERIOD_MS  */

#endif
