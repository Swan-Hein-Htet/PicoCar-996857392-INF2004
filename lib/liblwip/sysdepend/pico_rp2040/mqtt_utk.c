/*
 *	mqtt_utk.c
 *	MQTT client for the P5-IS20 PicoCar (RP2040, NO_SYS=1)
 *
 *	Runs entirely inside the kernel's CYW43/lwIP service task, which owns
 *	the radio and the whole network stack (TEAM.md section 3: no
 *	application task may call cyw43, lwIP or MQTT).  CHANGED FROM THE
 *	TEMPLATE by team agreement (TEAM.md IF-08/IF-09, docs/MQTT.md):
 *
 *	  p5is20/car/telemetry  QoS 0, the 21-byte binary record that T7
 *	                        leaves in mbf_telem, polled every 50 ms
 *	  p5is20/car/cmd        QoS 1 subscription, exactly one byte,
 *	                        forwarded to T7 through mbf_cmd
 *	  p5is20/car/status     retained "online"; last will "offline"
 *	  p5is20/car/log        QoS 0, one console line per message, taken
 *	                        from the application's log ring every 50 ms
 *
 *	Broker discovery: with MQTT_BROKER_IP "auto" (phone hotspot, where the
 *	PC's address changes) the car broadcasts "P5IS20 broker?" on UDP port
 *	18830 once a second; tools/mqtt/broker_finder.py on the PC answers and
 *	the reply's source address becomes the broker.  A failed connection
 *	starts the search again.  Phone hotspots do not always pass broadcasts
 *	between devices (bench 2026-11-02: the finder never heard the car), so
 *	after DISCOVERY_ASKS unanswered asks the car also tries every address
 *	of its subnet on the broker port, one after another (the PC on an
 *	iPhone hotspot is usually 172.20.10.2, the first one tried).
 *
 *	The application side is three non-blocking functions in
 *	app_program/subsystem1/net_bridge.c.  Nothing here ever waits on an
 *	application task and no application task ever waits on this code.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/netif.h"
#include <string.h>
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S    10
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_POLL_MS        50

#define TOPIC_TELEMETRY         "p5is20/car/telemetry"
#define TOPIC_CMD               "p5is20/car/cmd"
#define TOPIC_STATUS            "p5is20/car/status"
#define TOPIC_LOG               "p5is20/car/log"
#define TOPIC_DRIVE             "p5is20/car/drive"  /* manual: w a s d, space */
#define TOPIC_MAP               "p5is20/car/map"    /* barcode A-Z commands   */
#define LOG_LINE_MAX            256
#define LOG_LINES_PER_POLL      8       /* up to 160 lines/s */

#define DISCOVERY_PORT          18830
#define DISCOVERY_MS            1000
#define DISCOVERY_ASK           "P5IS20 broker?"
#define DISCOVERY_REPLY         "P5IS20 broker"
#define DISCOVERY_ASKS          3       /* then try addresses directly  */
#define SCAN_RETRY_MS           100
#define SCAN_MAX_HOSTS          254
#define STATUS_ONLINE           "online"
#define STATUS_OFFLINE          "offline"

#define RX_TOPIC_MAX            32
#define RX_DATA_MAX             32      /* commands one byte, map 26 */
#define TELEMETRY_MAX           32

/* Application bridge (app_program/subsystem1/net_bridge.c) */
extern int  net_bridge_take_telemetry(unsigned char *buf, int size);
extern void net_bridge_command(unsigned char const *data, int len);
extern void net_bridge_drive(unsigned char key);
extern void net_bridge_map(unsigned char const * p_data, int len);
extern void net_bridge_connected(int connected);
extern int  net_bridge_take_log(unsigned char *buf, int size);

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;
static u8_t		poll_armed;

static char		rx_topic[RX_TOPIC_MAX];
static u8_t		rx_data[RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_too_long;
static u8_t		tx_buf[TELEMETRY_MAX];
static u8_t		log_buf[LOG_LINE_MAX];
static u16_t		log_pending;	/* taken, not yet accepted by mqtt */

static u8_t		broker_known;
static u8_t		broker_auto;
static struct udp_pcb	*disc_pcb;
static u8_t		disc_armed;
static u32_t		disc_asks;	/* asks since the search began   */
static u32_t		scan_next;	/* host offset to try next, 0 = none */
static u8_t		scanning;	/* broker_addr came from the scan */

static int topic_is(const char *a, const char *b)
{
	while((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return (*a == *b);
}

static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	for(i = 0; i < (RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';
	rx_len = 0;
	rx_too_long = (tot_len > RX_DATA_MAX)? 1: 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;
	for(i = 0; i < len; i++) {
		if(rx_len < RX_DATA_MAX) {
			rx_data[rx_len++] = data[i];
		} else {
			rx_too_long = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		if(topic_is(rx_topic, TOPIC_CMD) && !rx_too_long) {
			/* Length is validated again on the application side:
			   anything that is not exactly one byte is rejected. */
			net_bridge_command(rx_data, (int)rx_len);
		} else if(topic_is(rx_topic, TOPIC_DRIVE) && (1 == rx_len)) {
			net_bridge_drive(rx_data[0]);
		} else if(topic_is(rx_topic, TOPIC_MAP)) {
			net_bridge_map(rx_data, rx_too_long ? 0 : (int)rx_len);
		} else {
			tm_printf((UB*)"mqtt: ignored %d bytes on [%s]\n",
				  (int)rx_len, rx_topic);
		}
	}
}

static void request_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: request failed, err=%d\n", (int)result);
	}
}

/* Console lines: a line the MQTT ring could not take stays pending and
   goes first next time, so nothing is lost or reordered here. */
static void publish_log(void)
{
	int	n;

	for(n = 0; n < LOG_LINES_PER_POLL; n++) {
		if(log_pending == 0) {
			log_pending = (u16_t)net_bridge_take_log(log_buf, LOG_LINE_MAX);
		}
		if(log_pending == 0) {
			break;
		}
		if(mqtt_publish(client, TOPIC_LOG, log_buf, log_pending,
				0 /* qos */, 0 /* retain */, NULL, NULL) != ERR_OK) {
			break;			/* ring full: retry next poll */
		}
		log_pending = 0;
	}
}

/* ---- broker discovery (MQTT_BROKER_IP "auto") ---- */

static void disc_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
		      const ip_addr_t *addr, u16_t port)
{
	char	msg[sizeof(DISCOVERY_REPLY)];

	(void)arg;
	(void)pcb;
	(void)port;
	if(p == NULL) {
		return;
	}
	if(!broker_known && p->tot_len == (u16_t)(sizeof(DISCOVERY_REPLY) - 1)) {
		(void)pbuf_copy_partial(p, msg, (u16_t)(sizeof(msg) - 1), 0);
		msg[sizeof(msg) - 1] = '\0';
		if(strcmp(msg, DISCOVERY_REPLY) == 0) {
			ip_addr_copy(broker_addr, *addr);
			broker_known = 1;
			scanning = 0;
			tm_printf((UB*)"mqtt: broker found at %s\n", ipaddr_ntoa(addr));
		}
	}
	pbuf_free(p);
}

static void disc_ask(void)
{
	struct pbuf	*p;

	p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(sizeof(DISCOVERY_ASK) - 1), PBUF_RAM);
	if(p == NULL) {
		return;
	}
	memcpy(p->payload, DISCOVERY_ASK, sizeof(DISCOVERY_ASK) - 1);
	(void)udp_sendto(disc_pcb, p, IP_ADDR_BROADCAST, DISCOVERY_PORT);
	/* Also the subnet's own broadcast (e.g. 172.20.10.15 on an iPhone
	   hotspot): some access points forward only one of the two. */
	if(netif_default != NULL) {
		ip_addr_t	subnet;
		u32_t const	ip = ip4_addr_get_u32(netif_ip4_addr(netif_default));
		u32_t const	mask = ip4_addr_get_u32(netif_ip4_netmask(netif_default));

		if(ip != 0) {
			ip_addr_set_ip4_u32(&subnet, ip | ~mask);
			(void)udp_sendto(disc_pcb, p, &subnet, DISCOVERY_PORT);
		}
	}
	pbuf_free(p);
}

static void disc_timeout(void *arg)
{
	(void)arg;
	disc_armed = 0;			/* next mqtt_utk_start() asks again */
}

/* Next address of our own subnet to try as the broker (host order inside,
   network order in the ip_addr_t).  Skips the network, broadcast and our
   own address; wraps round. */
static int scan_pick(void)
{
	u32_t	ip, mask, net, hosts, h;
	u32_t	tries;

	if(netif_default == NULL) {
		return 0;
	}
	ip = lwip_ntohl(ip4_addr_get_u32(netif_ip4_addr(netif_default)));
	mask = lwip_ntohl(ip4_addr_get_u32(netif_ip4_netmask(netif_default)));
	if(ip == 0) {
		return 0;
	}
	net = ip & mask;
	hosts = ~mask;			/* last offset = broadcast */
	if(hosts > SCAN_MAX_HOSTS + 1U) {
		hosts = SCAN_MAX_HOSTS + 1U;
	}
	for(tries = 0; tries < hosts; tries++) {
		h = scan_next;
		scan_next = (scan_next % (hosts - 1U)) + 1U;	/* 1 .. hosts-1 */
		if(h == 0 || (net | h) == ip) {
			continue;
		}
		ip_addr_set_ip4_u32(&broker_addr, lwip_htonl(net | h));
		return 1;
	}
	return 0;
}

static void discover(void)
{
	if(disc_pcb == NULL) {
		disc_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
		if(disc_pcb == NULL) {
			return;
		}
		ip_set_option(disc_pcb, SOF_BROADCAST);
		(void)udp_bind(disc_pcb, IP_ANY_TYPE, 0);
		udp_recv(disc_pcb, disc_recv, NULL);
		tm_printf((UB*)"mqtt: looking for the broker (UDP %d)\n", DISCOVERY_PORT);
	}
	if(!disc_armed) {
		disc_ask();
		disc_asks++;
		disc_armed = 1;
		sys_timeout(DISCOVERY_MS, disc_timeout, NULL);
	}
	if(disc_asks > DISCOVERY_ASKS) {
		if(scan_next == 0) {
			scan_next = 1;
			tm_printf((UB*)"mqtt: no answer to the search, trying"
				  " each address on port %d\n", MQTT_BROKER_PORT);
		}
		if(scan_pick()) {
			broker_known = 1;
			scanning = 1;
		}
	}
}

/* Every 50 ms: publish the newest record, if T7 left one. */
static void poll_timeout(void *arg)
{
	int	len;

	(void)arg;
	net_bridge_connected(mqtt_client_is_connected(client)? 1: 0);

	if(mqtt_client_is_connected(client)) {
		len = net_bridge_take_telemetry(tx_buf, TELEMETRY_MAX);
		if(len > 0) {
			(void)mqtt_publish(client, TOPIC_TELEMETRY, tx_buf,
					   (u16_t)len, 0 /* qos */, 0 /* retain */,
					   NULL, NULL);
		}
		publish_log();
	}
	sys_timeout(MQTT_UTK_POLL_MS, poll_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		connecting = 0;
		disc_asks = 0;
		scan_next = 0;
		scanning = 0;
		net_bridge_connected(1);
		tm_printf((UB*)"mqtt: connected to %s\n", ipaddr_ntoa(&broker_addr));

		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		(void)mqtt_subscribe(client, TOPIC_CMD, 1 /* qos */,
				     request_done_cb, NULL);
		(void)mqtt_subscribe(client, TOPIC_DRIVE, 0 /* qos */,
				     request_done_cb, NULL);
		(void)mqtt_subscribe(client, TOPIC_MAP, 1 /* qos */,
				     request_done_cb, NULL);
		(void)mqtt_publish(client, TOPIC_STATUS, STATUS_ONLINE,
				   (u16_t)(sizeof(STATUS_ONLINE) - 1), 1, 1,
				   request_done_cb, NULL);

		if(!poll_armed) {
			poll_armed = 1;		/* exactly one poll chain */
			sys_timeout(MQTT_UTK_POLL_MS, poll_timeout, NULL);
		}
	} else {
		net_bridge_connected(0);
		if(broker_auto) {
			broker_known = 0;	/* the PC may have a new address */
		}
		if(scanning) {
			/* Not this address: straight on to the next one. */
			sys_timeout(SCAN_RETRY_MS, retry_timeout, NULL);
		} else {
			tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
				  (int)status, MQTT_UTK_RETRY_MS / 1000);
			sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
		}
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!broker_known) {
		if(ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
			IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);
			broker_known = 1;		/* fixed address */
		} else {
			broker_auto = 1;		/* "auto": ask the PC */
			discover();
			return;
		}
	}

	client_info.client_id   = MQTT_CLIENT_ID;
	client_info.keep_alive  = MQTT_UTK_KEEPALIVE_S;
	client_info.will_topic  = TOPIC_STATUS;
	client_info.will_msg    = STATUS_OFFLINE;
	client_info.will_qos    = 1;
	client_info.will_retain = 1;

	if(scanning) {
		tm_printf((UB*)"mqtt: trying %s\n", ipaddr_ntoa(&broker_addr));
	}
	mqtt_disconnect(client);	/* clean state; no-op when closed */
	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		/* ERR_ISCONN (-10): the client was left mid-connection by the
		   previous attempt (bench 2026-11-02: every later attempt failed
		   at once and the log never reached the PC again).  Reset it;
		   no callback runs.  Then wait before the next attempt instead
		   of retrying on every poll. */
		mqtt_disconnect(client);
		if(broker_auto) {
			broker_known = 0;	/* search / scan on */
		}
		sys_timeout(scanning ? SCAN_RETRY_MS : MQTT_UTK_RETRY_MS,
			    retry_timeout, NULL);
		/* 'connecting' stays set until retry_timeout() clears it */
	}
}
