#ifndef __LORA_PKG_
#define __LORA_PKG_

#include "stdint.h"
#include "mesh_route.h"

typedef enum {
	TYPE_DATA = 0x0,
	TYPE_DATA_ACK,
	TYPE_PING,
	TYPE_RA,
} PkgType;

typedef enum {
	SUB_DATA = 0x0,
	SUB_CONTROL,
} subtypeData;

typedef enum {
	SUB_RA = 0x0,
	SUB_RA_RESPON,
	SUB_RA_FAIL,
} subtypeRA;

typedef enum {
	CMD_PING = 0x0,
	CMD_UPLOAD_PING,
	CMD_LED,
	CMD_GET_STAT,
	CMD_UPLOAD_STAT,
} cmdtype;

typedef enum {
	ACK_NO = 0x0,
	ACK,
} ackType;

#ifdef __CC_ARM
#pragma anon_unions
#endif

#pragma pack(1)

typedef struct {
	int8_t RssiPkt;
	int8_t SnrPkt;
	int8_t SignalRssiPkt;
} pkg_Status;

typedef struct {
	uint8_t src;
	uint8_t dst;
} LoRaMac;

typedef struct {
	uint8_t src;
	uint8_t dst;
	uint8_t pid;
	struct {
		uint8_t hop : 4;
		uint8_t subtype : 3;
		ackType ack : 1;
	};
} LoRaNet;

typedef struct {
	PkgType type;
	LoRaMac MacHeader;
	LoRaNet NetHeader;
} LoRaHeader;

typedef	union {
	struct {
		float temp;
		uint16_t volt;
	};
	uint8_t custom[6];
} AppPayload;

typedef struct {
	uint8_t RA_List[MAX_HOPS];
} *pRoutePayload, RoutePayload;

typedef struct {
	LoRaHeader Header;
	union {
		AppPayload AppData;
		RoutePayload RouteData;
	};
	pkg_Status stat; /* only use in Rx */
} *pLoRaPkg, LoRaPkg;

#pragma pack()

#define SIZE_HDR		(sizeof(LoRaHeader))

#define SIZE_PKG_MAX	SIZE_DATA
#define SIZE_DATA		((SIZE_HDR)+sizeof(AppPayload))
#define SIZE_RA			((SIZE_HDR)+sizeof(RoutePayload))
#define SIZE_PING		((SIZE_HDR)+(0))
#define SIZE_DATA_ACK	((SIZE_HDR)+(0))

#endif
