/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_mqtt_publisher_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "config.h"


/* Buffers for MQTT client. */
static uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

static struct pollfd fds[1];
static int nfds;

static bool connected;

#define GPIO_DATA_PIN 5
#define GPIO_NAME "foo"

atomic_t g_triggers = ATOMIC_INIT(0x0);
K_EVENT_DEFINE(motion_detected);



static void cooldown_expired(struct k_work *work)
{
	ARG_UNUSED(work);
	k_event_set(&motion_detected, 0x001);
}

static K_WORK_DELAYABLE_DEFINE(cooldown_work, cooldown_expired);

static void prepare_fds(struct mqtt_client *client)
{
	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds[0].fd = client->transport.tcp.sock;
	}

	fds[0].events = POLLIN;
	nfds = 1;
}

static void clear_fds(void)
{
	nfds = 0;
}

static int wait(int timeout)
{
	int ret = 0;

	if (nfds > 0) {
		ret = poll(fds, nfds, timeout);
		if (ret < 0) {
			LOG_ERR("poll error: %d", errno);
		}
	}

	return ret;
}

void mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		connected = true;
		LOG_INF("MQTT client connected!");

		break;

	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT client disconnected %d", evt->result);

		connected = false;
		clear_fds();

		break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBREC error %d", evt->result);
			break;
		}

		LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

		const struct mqtt_pubrel_param rel_param = {
			.message_id = evt->param.pubrec.message_id
		};

		err = mqtt_publish_qos2_release(client, &rel_param);
		if (err != 0) {
			LOG_ERR("Failed to send MQTT PUBREL: %d", err);
		}

		break;

	case MQTT_EVT_PUBCOMP:
		if (evt->result != 0) {
			LOG_ERR("MQTT PUBCOMP error %d", evt->result);
			break;
		}

		LOG_INF("PUBCOMP packet id: %u",
			evt->param.pubcomp.message_id);

		break;

	case MQTT_EVT_PINGRESP:
		LOG_INF("PINGRESP packet");
		break;

	default:
		break;
	}
}

static char *get_mqtt_payload(enum mqtt_qos qos)
{
	static char payload[64] = {0};
	snprintf(payload, sizeof(payload), "{\"distance\": %ld}", atomic_get(&g_triggers));

	return payload;
}

static char *get_mqtt_topic(void)
{
	return "homeassistant/sensor/catwheel/state";
}

static int publish(struct mqtt_client *client, enum mqtt_qos qos)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)get_mqtt_topic();
	param.message.topic.topic.size =
			strlen(param.message.topic.topic.utf8);
	param.message.payload.data = get_mqtt_payload(qos);
	param.message.payload.len =
			strlen(param.message.payload.data);
	param.message_id = sys_rand16_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
	LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))

static void broker_init(void)
{
	struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

	broker6->sin6_family = AF_INET6;
	broker6->sin6_port = htons(SERVER_PORT);
	inet_pton(AF_INET6, SERVER_ADDR, &broker6->sin6_addr);
}

static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
	client->client_id.size = strlen(MQTT_CLIENTID);
	client->password = NULL;
	client->user_name = NULL;
	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/* In this routine we block until the connected variable is 1 */
static int try_to_connect(struct mqtt_client *client)
{
	int rc, i = 0;

	while (i++ < APP_CONNECT_TRIES && !connected) {

		client_init(client);

		rc = mqtt_connect(client);
		if (rc != 0) {
			LOG_WRN("mqtt_connect: %d", rc);
			k_sleep(K_MSEC(APP_SLEEP_MSECS));
			continue;
		}

		prepare_fds(client);

		if (wait(APP_CONNECT_TIMEOUT_MS)) {
			mqtt_input(client);
		}

		if (!connected) {
			mqtt_abort(client);
		}
	}

	if (connected) {
		return 0;
	}

	return -EINVAL;
}

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
	int64_t remaining = timeout;
	int64_t start_time = k_uptime_get();
	int rc;

	while (remaining > 0 && connected) {
		if (wait(remaining)) {
			rc = mqtt_input(client);
			if (rc != 0) {
				PRINT_RESULT("mqtt_input", rc);
				return rc;
			}
		}

		rc = mqtt_live(client);
		if (rc != 0 && rc != -EAGAIN) {
			PRINT_RESULT("mqtt_live", rc);
			return rc;
		} else if (rc == 0) {
			rc = mqtt_input(client);
			if (rc != 0) {
				PRINT_RESULT("mqtt_input", rc);
				return rc;
			}
		}

		remaining = timeout + start_time - k_uptime_get();
	}

	return 0;
}

#define SUCCESS_OR_EXIT(rc) { if (rc != 0) { return 1; } }
#define SUCCESS_OR_BREAK(rc) { if (rc != 0) { break; } }

static int publisher(void)
{
	int i, rc, r = 0;

	LOG_INF("attempting to connect: ");
	rc = try_to_connect(&client_ctx);
	PRINT_RESULT("try_to_connect", rc);
	SUCCESS_OR_EXIT(rc);

	i = 0;
	while (i++ < CONFIG_NET_SAMPLE_APP_MAX_ITERATIONS && connected) {
		r = -1;

		rc = mqtt_ping(&client_ctx);
		PRINT_RESULT("mqtt_ping", rc);
		SUCCESS_OR_BREAK(rc);

		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
		SUCCESS_OR_BREAK(rc);

		rc = publish(&client_ctx, MQTT_QOS_0_AT_MOST_ONCE);
		PRINT_RESULT("mqtt_publish", rc);
		SUCCESS_OR_BREAK(rc);

		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
		if (rc==0)
		{
			LOG_INF("publish success");
			break;
		}

		r = 0;
	}

	rc = mqtt_disconnect(&client_ctx);
	PRINT_RESULT("mqtt_disconnect", rc);

	LOG_INF("Bye!");

	return r;
}

static int start_app(void)
{
	uint32_t  events;
	for (;;) {
			int r = 0, i = 0;
		LOG_INF("waiting..");
		events = k_event_wait(&motion_detected, 0xFFF, true, K_FOREVER);
		LOG_INF("got event");
		if (events == 0)
		{
			LOG_ERR("end of universe and no event :(");
		}
		LOG_INF("publishing data..");
		while (!CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS ||
		       i++ < CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS) {
			r = publisher();
	
			if (!CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS) {
				k_sleep(K_MSEC(5000));
			}
		}
		LOG_INF("done publishing..");
		uint32_t triggers = atomic_get(&g_triggers);
//		settings_save_one("foo/triggers", &triggers, sizeof(triggers));
		LOG_ERR("batt level %d", battery_level());

	}
	return 0;
}

static void button_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	atomic_inc(&g_triggers);
	k_work_reschedule(&cooldown_work, K_MSEC(3000));
}

static int foo_settings_set(const char *name, size_t len,
                            settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "triggers", &next) && !next) {
        if (len != sizeof(g_triggers)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &g_triggers, sizeof(g_triggers));
        if (rc >= 0) {
            return 0;
        }

        return rc;
    }


    return -ENOENT;
}

struct settings_handler my_conf = {
    .name = "foo",
    .h_set = foo_settings_set
};

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
	         DT_SPEC_AND_COMMA)
};

int32_t battery_level(void) {
    int err;
    int32_t val_mv;
    int16_t buf = 0;
    uint8_t level = 0; 

    struct adc_sequence sequence = {
	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),
    };

    if (!device_is_ready(adc_channels[0].dev)) {
	printk("ADC controller device not ready\n");
	return level;
    }
    err = adc_channel_setup_dt(&adc_channels[0]);
    if (err < 0) {
	printk("Could not setup channel #%d (%d)\n", 0, err);
	return level;
    }
    (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

    err = adc_read(adc_channels[0].dev, &sequence);
    if (err < 0) {
	printk("Could not read (%d)\n", err);
	return level; 
    }

    val_mv = buf;
    LOG_ERR("raw: %d", buf);
    err = adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);
    if (err < 0) {
	printk(" (value in mV not available)\n");
	return level; 
    }

    return val_mv;
}

int main(void)
{
	const struct device *gpio_dev;

	settings_subsys_init();
	settings_register(&my_conf);
	settings_load();

	low_power_enable();

	gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio_dev)) {
		printk("GPIO device %s is not ready!\n", gpio_dev->name);
		return 0;
	}

	int ret = gpio_pin_configure(gpio_dev, GPIO_DATA_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("Error configuring " GPIO_NAME "%d!\n", GPIO_DATA_PIN);
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, GPIO_DATA_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("error in interrupt conifgure\n");
		return ret;
	}

	static struct gpio_callback gpio_cb;

	gpio_init_callback(&gpio_cb, button_cb, BIT(GPIO_DATA_PIN));
	gpio_add_callback(gpio_dev, &gpio_cb);
//	wait_for_network();
//	while (true) {
//		LOG_ERR("batt level %d", battery_level());
//		k_sleep(K_MSEC(500));
//	}
	exit(start_app());
	return 0;
}
