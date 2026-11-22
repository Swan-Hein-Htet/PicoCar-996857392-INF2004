/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip).
   "auto": the car finds the PC on the network by itself (run
   tools/mqtt/start_broker.sh on the PC).  Use a fixed address such as
   "192.168.1.10" only if the PC always gets the same one.
   The Pico W joins 2.4 GHz WPA2-Personal networks only: on a phone
   hotspot turn on "Maximise compatibility" (iPhone) or pick 2.4 GHz. */
#define MQTT_BROKER_IP   "auto"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "p5is20-car"
/* Topics are fixed in lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c:
   p5is20/car/telemetry, p5is20/car/cmd, p5is20/car/status,
   p5is20/car/log (docs/MQTT.md) */
#define MQTT_TOPIC       "p5is20/car/status"

#endif /* WIFI_SECRETS_H */
