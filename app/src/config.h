#ifndef __CONFIG_H__
#define __CONFIG_H__

#define ZEPHYR_ADDR		"fdde:ad00:beef::5"
#define SERVER_ADDR		"fd00:dead:beef::2"
#define SERVER_PORT		1883

#define APP_CONNECT_TIMEOUT_MS	2000
#define APP_SLEEP_MSECS		500

#define APP_CONNECT_TRIES	10

#define APP_MQTT_BUFFER_SIZE	128

#define MQTT_CLIENTID		"zephyr_publisher"


#endif
