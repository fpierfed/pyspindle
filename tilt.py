from machine import I2C, Pin, RTC, DEEPSLEEP, deepsleep
import math
import network
import os
import time
from mqtt.robust import MQTTClient
import ujson
import mpu6050

# Constants
RAD_TO_DEG = 57.2957786
DEFAULTS = {
    'mpu6050_addr': 0x68,
    'scl_pin': 5,
    'sda_pin': 4,
}
CONFIG_FILE = 'config.json'


def parse_config_file(config_file, defaults=DEFAULTS):
    # TODO: recursive dict update
    config = dict(defaults)

    try:
        # No os.path.exists in micropython!
        os.stat(config_file)
    except OSError:
        return config

    with open(config_file) as f:
        config.update(ujson.load(f))
    return config


def setup_wifi(ssid, password):
    sta_if = network.WLAN(network.STA_IF)
    ap_if = network.WLAN(network.AP_IF)
    ap_if.active(False)

    if not sta_if.isconnected():
        sta_if.active(True)
        sta_if.connect(ssid, password)
        while not sta_if.isconnected():
            time.sleep_ms(50)


config = parse_config_file(CONFIG_FILE)
i2c = I2C(scl=Pin(config['scl_pin']),
          sda=Pin(config['sda_pin']),
          freq=400000)
mpu6050.MPU_Init(i2c, int(config['mpu6050_addr']))
setup_wifi(config['wifi']['ssid'], password=config['wifi']['password'])
client = MQTTClient(config['mqtt']['client_id'], config['mqtt']['broker'])

tilts = []
for _ in range(config['num_samples']):
    acc = mpu6050.read_accel_data(i2c, int(config['mpu6050_addr']))
    temp = mpu6050.read_temp_data(i2c, int(config['mpu6050_addr']))

    modulus = math.sqrt(sum([a * a for a in acc]))
    tilts.append(math.acos(acc[2] / modulus) * RAD_TO_DEG)
    time.sleep_ms(200)

tilts.sort()
if config['num_samples'] % 2 == 0:
    tilt = (tilts[config['num_samples'] // 2] +
            tilts[config['num_samples'] // 2 - 1]) / 2.
else:
    tilt = tilts[config['num_samples'] // 2]

client.connect()
client.publish(config['mqtt']['topic'] + '/tilt', b'%.02f' % (tilt))
client.publish(config['mqtt']['topic'] + '/temperature', b'%.02f' % (temp))
client.disconnect()
print('Tilt: %.2f  Temp: %.2f C' % (tilt, temp))

rtc = RTC()
rtc.irq(trigger=rtc.ALARM0, wake=DEEPSLEEP)
rtc.alarm(RTC.ALARM0, 1000 * config['sleep_time'])
deepsleep()
