#ifndef __BOARD_INIT_
#define __BOARD_INIT_
#include "stdint.h"

extern const uint8_t leds_list[];

void leds_init(void);
void saadc_init(void);

#endif
