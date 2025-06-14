#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(open_catwheel, LOG_LEVEL_DBG);

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include <sensor/hx711/hx711.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

int32_t measure(const struct device *scale_dev);

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

static const struct gpio_dt_spec hx711_pwr =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), hx711_power_gpios);

atomic_t g_triggers = ATOMIC_INIT(0x0);
int32_t g_weight = 0;
bool g_first_tare = true;

K_EVENT_DEFINE(motion_detected);

void scale_power(bool on) {
  if (on) {
    gpio_pin_set_dt(&hx711_pwr, 1);
    // datasheets says 400 ms typ
    k_sleep(K_MSEC(500));
  } else {
    gpio_pin_set_dt(&hx711_pwr, 0);
  }
}

K_MUTEX_DEFINE(scale_mutex);

void tare_work_handler(struct k_work *work) {
  const struct device *hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);
  LOG_INF("taring, first tare: %d", g_first_tare);

  if (k_mutex_lock(&scale_mutex, K_NO_WAIT) != 0) {
    // it is already measuring, skip tarring
    LOG_ERR("scale in use, aborting tare!");
    return;
  }

  int32_t cur_weight = measure(hx711_dev);
  if (cur_weight == 0) {
    LOG_ERR("0 weight skipping tarring!");
    goto out;
  }

  if (abs(cur_weight) > 2000 && !g_first_tare) {
    LOG_ERR("weight of %d detected, aborting tare!", cur_weight);
    goto out;
  }

  scale_power(1);
  uint32_t tare = avia_hx711_tare(hx711_dev, 5);
  LOG_INF("new auto tare = %d", tare);
  scale_power(0);
  if (g_first_tare)
    g_first_tare = false;
out:
  k_mutex_unlock(&scale_mutex);
}

static int cmd_tare(const struct shell *sh, size_t argc, char *argv[]) {
  scale_power(1);
  const struct device *hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);
  uint32_t tare = avia_hx711_tare(hx711_dev, 5);
  LOG_INF("manual new tare = %d", tare);
  scale_power(0);
  return 0;
}

K_WORK_DEFINE(tare_work, tare_work_handler);
static int cmd_autotare(const struct shell *sh, size_t argc, char *argv[]) {
  k_work_submit(&tare_work);
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    tare_command, SHELL_CMD(tare, NULL, "Tare the device\n", cmd_tare),
    SHELL_CMD(autotare, NULL, "Auto tare the device\n", cmd_autotare),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(scale, &tare_command, "trigger tare", NULL);

void tare_timer_handler(struct k_timer *dummy) { k_work_submit(&tare_work); }

K_TIMER_DEFINE(tare_timer, tare_timer_handler, NULL);

static void cooldown_expired(struct k_work *work) {
  ARG_UNUSED(work);
  k_event_set(&motion_detected, 0x001);
}

static K_WORK_DELAYABLE_DEFINE(cooldown_work, cooldown_expired);

static void prepare_fds(struct mqtt_client *client) {
  if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
    fds[0].fd = client->transport.tcp.sock;
  }

  fds[0].events = POLLIN;
  nfds = 1;
}

static void clear_fds(void) { nfds = 0; }

static int wait(int timeout) {
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
                      const struct mqtt_evt *evt) {
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
        .message_id = evt->param.pubrec.message_id};

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

    LOG_INF("PUBCOMP packet id: %u", evt->param.pubcomp.message_id);

    break;

  case MQTT_EVT_PINGRESP:
    LOG_INF("PINGRESP packet");
    break;

  default:
    break;
  }
}

static char *get_mqtt_payload(enum mqtt_qos qos) {
  static char payload[64] = {0};
  snprintf(payload, sizeof(payload), "{\"distance\": %ld, \"weight\": %i}",
           atomic_get(&g_triggers), g_weight);

  return payload;
}

static char *get_mqtt_topic(void) {
  return "homeassistant/sensor/catwheel/state";
}

static int publish(struct mqtt_client *client, enum mqtt_qos qos) {
  struct mqtt_publish_param param;

  param.message.topic.qos = qos;
  param.message.topic.topic.utf8 = (uint8_t *)get_mqtt_topic();
  param.message.topic.topic.size = strlen(param.message.topic.topic.utf8);
  param.message.payload.data = get_mqtt_payload(qos);
  param.message.payload.len = strlen(param.message.payload.data);
  param.message_id = sys_rand16_get();
  param.dup_flag = 0U;
  param.retain_flag = 0U;

  return mqtt_publish(client, &param);
}

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))

static void broker_init(void) {
  struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

  broker6->sin6_family = AF_INET6;
  broker6->sin6_port = htons(SERVER_PORT);
  inet_pton(AF_INET6, SERVER_ADDR, &broker6->sin6_addr);
}

static void client_init(struct mqtt_client *client) {
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
static int try_to_connect(struct mqtt_client *client) {
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

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout) {
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

#define SUCCESS_OR_EXIT(rc)                                                    \
  {                                                                            \
    if (rc != 0) {                                                             \
      return 1;                                                                \
    }                                                                          \
  }
#define SUCCESS_OR_BREAK(rc)                                                   \
  {                                                                            \
    if (rc != 0) {                                                             \
      break;                                                                   \
    }                                                                          \
  }

static int publisher(void) {
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
    if (rc == 0) {
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

static int start_app(void) {
  uint32_t events;
  for (;;) {
    int r = 0, i = 0;
    LOG_INF("waiting..");
    events = k_event_wait(&motion_detected, 0xFFF, true, K_FOREVER);
    LOG_INF("got event");
    if (events == 0) {
      LOG_ERR("end of universe and no event :(");
    }
    const struct device *hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);

    if (k_mutex_lock(&scale_mutex, K_NO_WAIT) == 0) {
       // it is not tarring
       g_weight = measure(hx711_dev);
       k_mutex_unlock(&scale_mutex);
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
    // settings_save_one("foo/triggers", &triggers, sizeof(triggers));
    LOG_ERR("batt level %d", battery_level());
  }
  return 0;
}

static void magnet_cb(const struct device *port, struct gpio_callback *cb,
                      gpio_port_pins_t pins) {
  atomic_inc(&g_triggers);
  k_work_reschedule(&cooldown_work, K_MSEC(3000));
}

static int foo_settings_set(const char *name, size_t len,
                            settings_read_cb read_cb, void *cb_arg) {
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

struct settings_handler my_conf = {.name = "foo", .h_set = foo_settings_set};

#define DT_SPEC_AND_COMMA(node_id, prop, idx)                                  \
  ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

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

static void gpio_output_voltage_setup(void) {
  // Configure UICR_REGOUT0 register only if it is set to default value.
  if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
      (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }

    NRF_UICR->REGOUT0 =
        (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
        (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }

    // System reset is needed to update UICR registers.
    NVIC_SystemReset();
  }
}

int32_t measure(const struct device *scale_dev) {
  static struct sensor_value weight;
  int ret = 0;

  scale_power(1);
  ret = sensor_sample_fetch(scale_dev);
  scale_power(0);
  if (ret != 0) {
    LOG_ERR("Cannot take measurement: %d", ret);
    return 0;
  }

  sensor_channel_get(scale_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
  LOG_INF("Weight: %d.%06d grams", weight.val1, weight.val2);
  return weight.val1;
}

int main(void) {
  const struct device *gpio_dev;

  gpio_output_voltage_setup();

  settings_subsys_init();
  settings_register(&my_conf);
  settings_load();

  low_power_enable();

  gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  if (!device_is_ready(gpio_dev)) {
    printk("GPIO device %s is not ready!\n", gpio_dev->name);
    return 0;
  }

  int ret =
      gpio_pin_configure(gpio_dev, GPIO_DATA_PIN, GPIO_INPUT | GPIO_PULL_UP);
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

  gpio_init_callback(&gpio_cb, magnet_cb, BIT(GPIO_DATA_PIN));
  gpio_add_callback(gpio_dev, &gpio_cb);

  gpio_pin_configure_dt(&hx711_pwr, GPIO_OUTPUT_INACTIVE);

  //	wait_for_network();
  //	while (true) {
  //		LOG_ERR("batt level %d", battery_level());
  //		k_sleep(K_MSEC(500));
  //	}
  k_timer_start(&tare_timer, K_SECONDS(10), K_HOURS(1));
  exit(start_app());
  return 0;
}
