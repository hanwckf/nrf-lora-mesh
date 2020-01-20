#include "mesh_route.h"
#include "compact.h"
#include "FreeRTOS.h"
#include "task.h"

static Addr_t _addr;
static RouteTableEntry _routes[ROUTING_TABLE_SIZE];
static LinkQualityEntry _LinkQuality[255];

#define DELETE_ROUTE(index) \
	do { \
		memcpy(&_routes[index], &_routes[index+1], \
			sizeof(RouteTableEntry) * (ROUTING_TABLE_SIZE - index - 1)); \
		_routes[ROUTING_TABLE_SIZE - 1].state = Invalid; \
	} while (0)

static void _delRouteByNexthop (uint8_t next_hop)
{
	taskENTER_CRITICAL();
	uint8_t i;
	for (i=0; i<ROUTING_TABLE_SIZE; i++) {
		if (_routes[i].next_hop == next_hop 
				&& _routes[i].state == Valid)
			_routes[i].state = Invalid;
	}
	taskEXIT_CRITICAL();
}

static void _updateLinkQualityMap (uint8_t addr, int16_t quality)
{
	taskENTER_CRITICAL();
	_LinkQuality[addr].valid = true;
	_LinkQuality[addr].quality = quality;
	taskEXIT_CRITICAL();
}

static void _delRouteByDest (uint8_t dest)
{
	taskENTER_CRITICAL();
	uint8_t i;
	for (i=0; i<ROUTING_TABLE_SIZE; i++) {
		if (_routes[i].dest == dest 
				&& _routes[i].state == Valid)
			_routes[i].state = Invalid;
	}
	taskEXIT_CRITICAL();
}

#define RSSI_WEAK2_THRESHOLD -100
#define RSSI_DIFF2_THRESHOLD 10

#define RSSI_WEAK1_THRESHOLD -80
#define RSSI_DIFF1_THRESHOLD 20

#define RSSI_WEAK0_THRESHOLD -60
#define RSSI_DIFF0_THRESHOLD 30

static bool isNeedUpdate(uint8_t old_hop, uint8_t new_hop)
{
	if (!(_LinkQuality[new_hop].valid && _LinkQuality[old_hop].valid))
		return true;

	if (_LinkQuality[old_hop].quality < RSSI_WEAK2_THRESHOLD
			&& _LinkQuality[new_hop].quality - _LinkQuality[old_hop].quality > RSSI_DIFF2_THRESHOLD) {
			return true;
	} else if (_LinkQuality[old_hop].quality < RSSI_WEAK1_THRESHOLD
			&& _LinkQuality[new_hop].quality - _LinkQuality[old_hop].quality > RSSI_DIFF1_THRESHOLD) {
			return true;
	} else if (_LinkQuality[old_hop].quality < RSSI_WEAK0_THRESHOLD
			&& _LinkQuality[new_hop].quality - _LinkQuality[old_hop].quality > RSSI_DIFF0_THRESHOLD) {
			return true;
	}
	return false;
}

static void _updateRoute (uint8_t dest, uint8_t next_hop, uint8_t hops)
{
	taskENTER_CRITICAL();
	uint8_t i;
	do {
		for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		{
			if (_routes[i].dest == dest && _routes[i].state == Valid
					&& isNeedUpdate(_routes[i].next_hop, next_hop))
			{
				_routes[i].next_hop = next_hop;
				_routes[i].hops = hops;
				break;
			}
		}

		for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		{
			if (_routes[i].state == Invalid)
			{
				_routes[i].dest = dest;
				_routes[i].next_hop = next_hop;
				_routes[i].state = Valid;
				_routes[i].hops = hops;
				break;
			}
		}

		// Need to make room for a new one
		DELETE_ROUTE(0);
		// Should be an invalid slot now
		for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		{
			if (_routes[i].state == Invalid)
			{
				_routes[i].dest = dest;
				_routes[i].next_hop = next_hop;
				_routes[i].state = Valid;
				_routes[i].hops = hops;
				break;
			}
		}
	} while (0);
	taskEXIT_CRITICAL();
}

static int8_t _getRouteTo (uint8_t dest) {
	uint8_t i;
	for (i = 0; i<ROUTING_TABLE_SIZE; i++) {
		if (_routes[i].dest == dest && _routes[i].state != Invalid) {
				return _routes[i].next_hop;
		}
	}
	return -1; /* No route to host */
}

static uint8_t _getNetAddr (void) {
	return _addr.net;
}

static uint8_t _getMacAddr (void) {
	return _addr.mac;
}

static void _clearRoutingTable (void) {
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		_routes[i].state = Invalid;
}

static void _initRouteTable (Addr_t *addr) {
	_addr.mac = addr->mac;
	_addr.net = addr->net;
	Route.clearRoutingTable();
}

const MeshRoute_t Route = {
	.getRouteTo = _getRouteTo,
	.initRouteTable = _initRouteTable,
	.clearRoutingTable = _clearRoutingTable,
	.updateRoute = _updateRoute,
	.getNetAddr = _getNetAddr,
	.getMacAddr = _getMacAddr,
	.delRouteByNexthop = _delRouteByNexthop,
	.delRouteByDest = _delRouteByDest,
	.updateLinkQualityMap = _updateLinkQualityMap,
};
