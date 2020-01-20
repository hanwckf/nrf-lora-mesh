#ifndef __MESH_ROUTE_
#define __MESH_ROUTE_

#include <stdint.h>
#include <stdbool.h>

#define ROUTING_TABLE_SIZE 16

#define MAX_HOPS 8
#define TTL_MAX MAX_HOPS

typedef enum {
	Invalid = 0,
	Discovering,
	Valid,
} RouteState;

typedef struct {
	uint8_t mac;
	uint8_t net;
} Addr_t;

typedef struct {
	uint8_t dest;
	uint8_t next_hop;
	RouteState state;
	uint8_t hops;
} RouteTableEntry;

typedef struct {
	int16_t quality;
	bool valid;
} LinkQualityEntry;

typedef struct {
	int8_t (*getRouteTo) (uint8_t dst);
	void (*updateRoute) (uint8_t dest, uint8_t next_hop, uint8_t hops);
	void (*updateLinkQualityMap) (uint8_t addr, int16_t quality);
	void (*initRouteTable) (Addr_t *addr);
	void (*clearRoutingTable) (void);
	uint8_t (*getNetAddr) (void);
	uint8_t (*getMacAddr) (void);
	void (*delRouteByNexthop) (uint8_t next_hop);
	void (*delRouteByDest) (uint8_t dest);
} MeshRoute_t;

extern const MeshRoute_t Route;

#endif
