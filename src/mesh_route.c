#define DEBUG_LOG
#include "mesh_route.h"
#include "compact.h"
#include "FreeRTOS.h"
#include "task.h"

static Addr_t _addr;
RouteTableEntry _routes[ROUTING_TABLE_SIZE];
LinkQualityEntry _LinkQuality[255];

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
	NRF_LOG_DBG("DEL nh: 0x%02x", next_hop);
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

static bool isNeedUpdate(uint8_t old_hop, uint8_t new_hop, uint8_t old_hop_count, uint8_t new_hop_count)
{
	int16_t i,j;

	if (!(_LinkQuality[new_hop].valid && _LinkQuality[old_hop].valid)) {
		NRF_LOG_DBG("Y0");
		return true;
	}
	
	i = _LinkQuality[old_hop].quality; j = _LinkQuality[new_hop].quality;
	NRF_LOG_DBG("oldhop: 0x%02x(%d), newhop: 0x%02x(%d)", old_hop, i, new_hop, j);

	if (i < RSSI_WEAK2_THRESHOLD
			&& j - i > RSSI_DIFF2_THRESHOLD) {
			NRF_LOG_DBG("Y1");
			return true;
	} else if (i < RSSI_WEAK1_THRESHOLD
			&& j - i > RSSI_DIFF1_THRESHOLD) {
			NRF_LOG_DBG("Y2");
			return true;
	} else if (i < RSSI_WEAK0_THRESHOLD
			&& j - i > RSSI_DIFF0_THRESHOLD) {
			NRF_LOG_DBG("Y3");
			return true;
	} else if (j > RSSI_WEAK0_THRESHOLD && new_hop_count < old_hop_count) {
		NRF_LOG_DBG("Y4");
		return true;
	}
	NRF_LOG_DBG("N0");
	return false;
}

static void _updateRoute (uint8_t dest, uint8_t next_hop, uint8_t hops)
{
	taskENTER_CRITICAL();
	uint8_t i;

	if (dest == Route.getNetAddr())
		goto out;
	
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if ( _routes[i].dest == dest && _routes[i].state == Valid) 
		{
			if (_routes[i].next_hop == next_hop) {
				_routes[i].hops = hops;
				goto out; /* It is the same Route entry */
			}
			
			if ( isNeedUpdate(_routes[i].next_hop, next_hop, _routes[i].hops, hops)) {
				NRF_LOG_DBG("UPDATE! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);
				_routes[i].next_hop = next_hop;
				_routes[i].hops = hops;
			}
			goto out;
		}
	}

	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			NRF_LOG_DBG("NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);
			_routes[i].dest = dest;
			_routes[i].next_hop = next_hop;
			_routes[i].state = Valid;
			_routes[i].hops = hops;
			goto out;
		}
	}

	// Need to make room for a new one
	DELETE_ROUTE(0);
	// Should be an invalid slot now
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			NRF_LOG_DBG("NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);
			_routes[i].dest = dest;
			_routes[i].next_hop = next_hop;
			_routes[i].state = Valid;
			_routes[i].hops = hops;
			goto out;
		}
	}
out:
	taskEXIT_CRITICAL();
}

static int8_t _getRouteTo (uint8_t dest) {
	uint8_t i;
	for (i = 0; i<ROUTING_TABLE_SIZE; i++) {
		if (_routes[i].dest == dest && _routes[i].state != Invalid) {
				NRF_LOG_DBG("Route found! dst: 0x%02x, nh: 0x%02x", dest, _routes[i].next_hop);
				return _routes[i].next_hop;
		}
	}
	NRF_LOG_DBG("Route fail! dst: 0x%02x", dest);
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

static void _clearLinkQuailtyMap(void) {
	uint8_t i;
	for (i = 0; i< 255; i++) {
		_LinkQuality[i].valid = false;
	}
}

static void _initRouteTable (Addr_t *addr) {
	_addr.mac = addr->mac;
	_addr.net = addr->net;
	_clearLinkQuailtyMap();
	_clearRoutingTable();
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
