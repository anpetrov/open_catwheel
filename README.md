# Open Catwheel run detector

Detect 'onefastcat' wheel turning, and publish MQTT message with number of rotations.
Uses Thread for communication, operates as Sleepy End Device. Consumes 30uA in sleep mode,
with 1 sec poll interval.

## building
```
west build -b ebyte_e73_custom/nrf52840 open_catwheel/app
```
## flashing
```
flash -d zephyr/build/ -r pyocd
```

## Setting with Home Assistant

The sensor publishes number of detections on `homeassistant/sensor/catwheel/state` topic.
HA can auto-create mqtt sensors, so the following example configuration can be used:

```
  mosquitto_pub -r -h MQTT_SERVER_IP -p 1883 -t "homeassistant/sensor/catwheelD/config" -f
  {
    "device_class":"distance",
    "state_topic":"homeassistant/sensor/catwheel/state",
    "unit_of_measurement":"m",
    "value_template":"{{ value_json.distance}}",
    "unique_id":"cat_wheel_01",
    "device":{
       "identifiers":[
           "cat_wheel_01"
       ],
       "name":"cat wheel"
    }
 }

```
Also, separate sensor for weight:
```
  mosquitto_pub -r -h MQTT_SERVER_IP -p 1883 -t "homeassistant/sensor/catwheelW/config" -f f1
  {
    "device_class":"weight",
    "state_topic":"homeassistant/sensor/catwheel/state",
    "unit_of_measurement":"g",
    "value_template":"{{ value_json.weight}}",
    "unique_id":"cat_wheel_02",
    "device":{
       "identifiers":[
           "cat_wheel_01"
       ],
       "name":"cat wheel"
    }
}
```

## OTA

To update:
```
~/go/bin/mcumgr --conntype udp --connstring=[fdde:ad00:beef::4]:1337 image upload ./foo.bin
```
Then confirm:

```
~/go/bin/mcumgr --conntype udp --connstring=[fdde:ad00:beef::4]:1337 image confirm 46ee8dd9107a71aaf828c3a8030e247ad387d993999aa93a1b4af5411466c8c4
```

Finally, reset:

```
~/go/bin/mcumgr --conntype udp --connstring=[fdde:ad00:beef::4]:1337 reset
```
