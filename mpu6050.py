from machine import I2C, Pin
import time
import math


# Constants
radToDeg = 57.2957786
DeviceAddress = 0x68   # MPU6050 device address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
TEMP_OUT_H = 0X41

calib_x_accel = 0.
calib_y_accel = 0.
calib_z_accel = 0.


def MPU_Init(i2c):
    # write to sample rate register
    i2c.writeto_mem(DeviceAddress, SMPLRT_DIV, b'\x07')

    # Write to power management register
    i2c.writeto_mem(DeviceAddress, PWR_MGMT_1, b'\x01')

    # Write to Configuration register
    # Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise
    # due to vibration.)
    # https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
    # int('0000110', 2) = 6
    i2c.writeto_mem(DeviceAddress, CONFIG, b'\x06')

    # Write to Gyro & Accel configuration registers to self test.
    i2c.writeto_mem(DeviceAddress, GYRO_CONFIG, b'\x18')
    i2c.writeto_mem(DeviceAddress, ACCEL_CONFIG, b'\x18')

    # Write to interrupt: 1 to enable interrupts, 0 to disable them
    i2c.writeto_mem(DeviceAddress, INT_ENABLE, b'\x01')


def read_raw_data(i2c, addr):
    # Accelero and Gyro value are 16-bit
    high = i2c.readfrom_mem(DeviceAddress, addr, 1)
    low = i2c.readfrom_mem(DeviceAddress, addr + 1, 1)

    # concatenate higher and lower value
    value = high[0] << 8 | low[0]

    # to get signed value from mpu6050
    if(value > 32768):
        value -= 65536
    return value


def calibrate(i2c):
    global calib_x_accel
    global calib_y_accel
    global calib_z_accel

    x_accel = 0
    y_accel = 0
    z_accel = 0

    # Discard the first set of values read from the IMU
    read_raw_data(i2c, ACCEL_XOUT_H)
    read_raw_data(i2c, ACCEL_YOUT_H)
    read_raw_data(i2c, ACCEL_ZOUT_H)

    # Read and average the raw values from the IMU
    for int in range(10):
        x_accel += read_raw_data(i2c, ACCEL_XOUT_H)
        y_accel += read_raw_data(i2c, ACCEL_YOUT_H)
        z_accel += read_raw_data(i2c, ACCEL_ZOUT_H)
        time.sleep_ms(100)

    x_accel /= 10
    y_accel /= 10
    z_accel /= 10

    # Store the raw calibration values globally
    calib_x_accel = x_accel
    calib_y_accel = y_accel
    calib_z_accel = z_accel
    print(calib_x_accel, calib_y_accel, calib_z_accel)


i2c = I2C(scl=Pin(5), sda=Pin(4), freq=400000)
MPU_Init(i2c)
calibrate(i2c)

while True:
    # We do not rescale the z component: at rest on a slat surface we expect
    # the x and y component of acceleration to be 0. We expect the z component
    # to be 1g exactly.
    acc = [
        read_raw_data(i2c, ACCEL_XOUT_H) - calib_x_accel,
        read_raw_data(i2c, ACCEL_YOUT_H) - calib_y_accel,
        read_raw_data(i2c, ACCEL_ZOUT_H),
    ]
    temp = read_raw_data(i2c, TEMP_OUT_H) / 340 + 36.53

    modulus = math.sqrt(sum([a * a for a in acc]))
    tilt = math.acos(acc[2] / modulus) * radToDeg

    print('Tilt: %.2f  Temp: %.2f C' % (tilt, temp))
    time.sleep_ms(500)
