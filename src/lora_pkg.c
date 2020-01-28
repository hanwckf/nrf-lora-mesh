#include "lora_pkg.h"
#include "stdint.h"

const int8_t pkgSizeMap[][2] = {
	{TYPE_DATA,			SIZE_DATA},
	{TYPE_DATA_ACK,		SIZE_DATA_ACK},
	{TYPE_PING,			SIZE_PING},
	{TYPE_RA,			SIZE_RA},
	{TYPE_MAX,			-1},
};
