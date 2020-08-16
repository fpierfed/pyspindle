# Constants
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H = 0X41


def MPU_Init(i2c, device_addr):
    # Use x gyro as clock source
    i2c.writeto_mem(device_addr, PWR_MGMT_1, b'\x01')

    # Set full scale accel range.
    i2c.writeto_mem(device_addr, ACCEL_CONFIG, b'\x00')

    # Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise
    # due to vibration.)
    # https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
    # int('0000110', 2) = 6
    # This means 19ms delay and sample rate of 5Hz
    i2c.writeto_mem(device_addr, CONFIG, b'\x06')


def read_raw_data(i2c, device_addr, addr):
    # Accelero and Gyro value are 16-bit
    high = i2c.readfrom_mem(device_addr, addr, 1)
    low = i2c.readfrom_mem(device_addr, addr + 1, 1)

    # concatenate higher and lower value
    value = high[0] << 8 | low[0]

    # to get signed value from mpu6050
    if(value > 32768):
        value -= 65536
    return value


def read_accel_data(i2c, device_addr):
    return [
        read_raw_data(i2c, device_addr, ACCEL_XOUT_H),
        read_raw_data(i2c, device_addr, ACCEL_YOUT_H),
        read_raw_data(i2c, device_addr, ACCEL_ZOUT_H)
    ]


def read_temp_data(i2c, device_addr):
    return read_raw_data(i2c, device_addr, TEMP_OUT_H) / 340. + 36.53
