#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time, sys, ctypes, struct, math, smbus
import RPi.GPIO as GPIO

SHT_DATA_PIN          = 24
SHT_CLOCK_PIN         = 23
MOTION_SENSOR_PIN     = 4
LED_PIN               = 18
# constants for addressing the bmp085/150 device
BMP_I2C_BUS           = 0
BMP_I2C_ADDRESS       = 0x77
# BMP calibration data
BMP_IC2_CAL_AC1       = 0xAA
BMP_IC2_CAL_AC2       = 0xAC
BMP_IC2_CAL_AC3       = 0xAE
BMP_IC2_CAL_AC4       = 0xB0
BMP_IC2_CAL_AC5       = 0xB2
BMP_IC2_CAL_AC6       = 0xB4
BMP_IC2_CAL_B1        = 0xB6
BMP_IC2_CAL_B2        = 0xB8
BMP_IC2_CAL_MB        = 0xBA
BMP_IC2_CAL_MC        = 0xBC
BMP_IC2_CAL_MD        = 0xBE
# the commands to BMP read sensor data
BMP_I2C_CTRL          = 0xF4
BMP_I2C_TEMP_CMD      = 0x2E
BMP_I2C_TEMP_DATA     = 0xF6
BMP_I2C_TEMP_CMD      = 0x2E
BMP_I2C_PRESSURE_CMD  = 0x34
BMP_I2C_PRESSURE_DATA = 0xF6
# BMP sampling setting
# 0                   = ultra low power
# 1                   = standard
# 2                   = high
# 3                   = ultra high resolution
BMP_RESOLUTION        = 3
# delays for oversampling settings 0, 1, 2 and 3
BMP_DELAYS            = [5, 8, 14, 26]


def setup():
    GPIO.setmode(GPIO.BCM)
    setup_motion_sensor(MOTION_SENSOR_PIN)


def setup_motion_sensor(input_pin):
    GPIO.setup(input_pin, GPIO.IN)


def motion_sensor(pin):
    return GPIO.input(pin)


def clock_tick(clock_pin, level, delay=0.0000001):
    GPIO.output(clock_pin, level)
    time.sleep(delay)


def shift_out_byte(data_pin, clock_pin, value, lsb_first=True, bits=8, delay=0.0000001):
    for i in xrange(0, bits):
        if lsb_first:
            GPIO.output(data_pin, bool(value & (1 << i)))
        else:
            GPIO.output(data_pin, bool(value & (1 << ((bits - 1 - i )))))

        clock_tick(clock_pin, GPIO.HIGH)
        clock_tick(clock_pin, GPIO.LOW)


def sht_send_command(data_pin, clock_pin, command):
    GPIO.setup(data_pin, GPIO.OUT)
    GPIO.setup(clock_pin, GPIO.OUT)

    GPIO.output(data_pin, GPIO.HIGH)
    clock_tick(clock_pin, GPIO.HIGH)

    GPIO.output(data_pin, GPIO.LOW)
    clock_tick(clock_pin, GPIO.LOW)
    clock_tick(clock_pin, GPIO.HIGH)
    GPIO.output(data_pin, GPIO.HIGH)
    clock_tick(clock_pin, GPIO.LOW)

    shift_out_byte(data_pin, clock_pin, command, False)
    GPIO.output(clock_pin, GPIO.HIGH)

    GPIO.setup(data_pin, GPIO.IN)
    ack = GPIO.input(data_pin)
    if ack != GPIO.LOW:
        raise Exception("SHT15 command ACK failure 1")

    GPIO.output(clock_pin, GPIO.LOW)
    ack = GPIO.input(data_pin)
    if ack != GPIO.HIGH:
        raise Exception("SHT15 command ACK failure 2")


def sht_wait_result(data_pin):
    GPIO.setup(data_pin, GPIO.IN)

    # wait for the SHTx answer
    # need to wait up to 2 seconds for the value
    ack = GPIO.HIGH
    for i in xrange(0, 100):
        time.sleep(.01)
        ack = GPIO.input(data_pin)
        if ack == GPIO.LOW:
            break

    if ack == GPIO.HIGH:
        raise Exception("SHT15 not responding with data")


def shift_in_byte(data_pin, clock_pin, lsb_first = True):
    GPIO.setup(data_pin, GPIO.IN)
    GPIO.setup(clock_pin, GPIO.OUT)

    value = 0x00
    for i in xrange(0, 8):
        GPIO.output(clock_pin, GPIO.HIGH)

        if lsb_first:
            value |= GPIO.input(data_pin) << i
        else:
            value |= GPIO.input(data_pin) << (7- i)

        GPIO.output(clock_pin, GPIO.LOW)

    return value


def sht_get_data(data_pin, clock_pin):
    # get data from the SHTx sensor
    # get the MSB (most significant bits)
    GPIO.setup(data_pin, GPIO.IN)
    GPIO.setup(clock_pin, GPIO.OUT)

    MSB = shift_in_byte(data_pin, clock_pin, False)

    # send the required ACK
    GPIO.setup(data_pin, GPIO.OUT)
    GPIO.output(data_pin, GPIO.HIGH)
    GPIO.output(data_pin, GPIO.LOW)
    GPIO.output(clock_pin, GPIO.HIGH)
    GPIO.output(clock_pin, GPIO.LOW)

    # get the LSB (less significant bits)
    GPIO.setup(data_pin, GPIO.IN)
    LSB = shift_in_byte(data_pin, clock_pin, False)
    return ((MSB << 8) | LSB)  # combine bits


def sht_skip_crc(data_pin, clock_pin):
    # skip CRC data from the SHTx sensor
    GPIO.setup(data_pin, GPIO.OUT)
    GPIO.setup(clock_pin, GPIO.OUT)
    GPIO.output(data_pin, GPIO.HIGH)
    clock_tick(clock_pin, GPIO.HIGH)
    clock_tick(clock_pin, GPIO.LOW)


def sht_get_temp(data_pin, clock_pin):
    # temp in celsius
    sht_send_command(data_pin, clock_pin, int(0x03))
    sht_wait_result(data_pin)

    val = sht_get_data(data_pin, clock_pin)
    sht_skip_crc(data_pin, clock_pin)
    return val * 0.01 - 40 # convert to celsius


def sht_get_humidity(data_pin, clock_pin, temp):
    # relative Humidity
    sht_send_command(data_pin, clock_pin, int(0x05))
    sht_wait_result(data_pin)

    raw_humidity = sht_get_data(data_pin, clock_pin)
    sht_skip_crc(data_pin, clock_pin)

    C1 = -2.0468
    C2 = 0.0367
    C3 = -1.5955e-6
    T1 = 0.01
    T2 = 0.00008
    # rh = raw_humidity from sensor
    # linear_humidty = c1 + c2 * rh + c3 * rh^2
    # temp_comp_humid = (temp - 25) * (t1 + t2 * rh) + linear_humidity

    linear_humidty = C1 + C2 * raw_humidity + C3 * raw_humidity * raw_humidity
    temp_comp_humid = (temp - 25.0 ) * (T1 + T2 * raw_humidity) + linear_humidty
    return temp_comp_humid


def calc_dew_point(temp, relative_humidity):
    # H = (log10(relative_humidity)-2)/0.4343 + (17.62*temp)/(243.12+temp);
    # DP = 243.12*H/(17.62-H);
    H = (math.log10(relative_humidity) - 2) / 0.4343 + (17.62 * temp) / (243.12 + temp)
    DP = 243.12 * H / (17.62 - H)
    return DP


def ic2_read_u16(bus, address, register):
    hi = bus.read_byte_data(address, register)
    lo = bus.read_byte_data(address, register + 1)
    return (hi << 8) + lo

def ic2_read_s8(bus, address, register):
    result = bus.read_byte_data(address, register)
    if result > 127:
        result -= 256
    return result

def ic2_read_s16(bus, address, register):
    hi = ic2_read_s8(bus, address, register)
    lo = bus.read_byte_data(address, register + 1)
    return (hi << 8) + lo


def bmp_read_calibration_data(bus):
    "Reads the calibration data from the IC"
    data        = {}
    data["AC1"] = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC1) # INT16
    data["AC2"] = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC2) # INT16
    data["AC3"] = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC3) # INT16
    data["AC4"] = ic2_read_u16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC4) # UINT16
    data["AC5"] = ic2_read_u16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC5) # UINT16
    data["AC6"] = ic2_read_u16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_AC6) # UINT16
    data["B1"]  = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_B1)  # INT16
    data["B2"]  = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_B2)  # INT16
    data["MB"]  = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_MB)  # INT16
    data["MC"]  = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_MC)  # INT16
    data["MD"]  = ic2_read_s16(bus, BMP_I2C_ADDRESS, BMP_IC2_CAL_MD)  # INT16
    return data


def bmp_canned_calibration_data():
    data = {}
    data["AC1"] = 408
    data["AC2"] = -72
    data["AC3"] = -14383
    data["AC4"] = 32741
    data["AC5"] = 32757
    data["AC6"] = 23153
    data["B1"]  = 6190
    data["B2"]  = 4
    data["MB"]  = -32768;
    data["MC"]  = -8711
    data["MD"]  = 2868
    return data


def bmp_print_calibration_data(data):
    for key, value in data.items():
        print "%s => %s" % (repr(key), repr(value))


def bmp_read_raw_temp(bus):
    bus.write_byte_data(BMP_I2C_ADDRESS, BMP_I2C_CTRL, BMP_I2C_TEMP_CMD)
    time.sleep(0.005) # sleep 5ms
    raw_temp = ic2_read_u16(bus, BMP_I2C_ADDRESS, BMP_I2C_TEMP_DATA)
    return raw_temp


def bmp_read_raw_pressure(bus):
    bus.write_byte_data(BMP_I2C_ADDRESS, BMP_I2C_CTRL, BMP_I2C_PRESSURE_CMD + (BMP_RESOLUTION << 6))

    time.sleep(BMP_DELAYS[BMP_RESOLUTION] / 1000.0)

    msb  = bus.read_byte_data(BMP_I2C_ADDRESS, BMP_I2C_PRESSURE_DATA)
    lsb  = bus.read_byte_data(BMP_I2C_ADDRESS, BMP_I2C_PRESSURE_DATA + 1)
    xlsb = bus.read_byte_data(BMP_I2C_ADDRESS, BMP_I2C_PRESSURE_DATA + 2)
    raw  = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - BMP_RESOLUTION)
    return raw


def bmp_adjust_temp(cal_data, UT):
    AC5 = cal_data["AC5"]
    AC6 = cal_data["AC6"]
    MC  = cal_data["MC"]
    MD  = cal_data["MD"]

    X1   = (UT - AC6) * AC5 >> 15
    X2   = (MC << 11) / (X1 + MD)
    B5   = X1 + X2
    temp = (B5 + 8) >> 4
    return B5, temp


def bmp_adjust_pressure(cal_data, B5, UP):
    AC1 = cal_data["AC1"]
    AC2 = cal_data["AC2"]
    AC3 = cal_data["AC3"]
    AC4 = cal_data["AC4"]
    B1  = cal_data["B1"]
    B2  = cal_data["B2"]

    B6  = B5 - 4000
    X1  = (B2 * (B6 * B6) >> 12) >> 11
    X2  = (AC2 * B6) >> 11
    X3  = X1 + X2
    B3  = (((AC1 * 4 + X3) << BMP_RESOLUTION) + 2) / 4
    X1  = (AC3 * B6) >> 13
    X2  = (B1 * ((B6 * B6) >> 12)) >> 16
    X3  = ((X1 + X2) + 2) >> 2
    B4  = (AC4 * (X3 + 32768)) >> 15
    B7  = (UP - B3) * (50000 >> BMP_RESOLUTION)

    if (B7 < 0x80000000):
      p = (B7 * 2) / B4
    else:
      p = (B7 / B4) * 2

    X1 = (p >> 8) * (p >> 8)
    X1 = (X1 * 3038) >> 16
    X2 = (-7357 * p) >> 16

    p = p + ((X1 + X2 + 3791) >> 2)
    return p


def get_altitude(p, sea_level=101325):
    return 44330.0 * (1.0 - pow(float(p) / sea_level, 0.1903))


def celsius_to_fahrenheit(x):
    return (x * (9.0 / 5.0)) + 32


def pascals_to_inches_mercury(x):
    return x * 2.96e-4


def main():
    try:
        setup()
        bus = smbus.SMBus(1) # the i2c bus
        bmp_cal_data = bmp_read_calibration_data(bus)

        while True:
            motion           = motion_sensor(MOTION_SENSOR_PIN);
            temp             = sht_get_temp(SHT_DATA_PIN, SHT_CLOCK_PIN)
            time.sleep(1)
            humidity         = sht_get_humidity(SHT_DATA_PIN, SHT_CLOCK_PIN, temp)
            dew_point        = calc_dew_point(temp, humidity)
            bmp_raw_temp     = bmp_read_raw_temp(bus)
            B5, bmp_temp     = bmp_adjust_temp(bmp_cal_data, bmp_raw_temp)
            bmp_raw_pressure = bmp_read_raw_pressure(bus)
            bmp_pressure     = bmp_adjust_pressure(bmp_cal_data, B5, bmp_raw_pressure)

            sys.stdout.write("motion: %s | temp: %06.2f°F | relative humidity: %06.2f | dew point: %06.2f°F | pressure: %06.2f inHg | altitude: %06.2fm\r" %
                             (motion,
                              celsius_to_fahrenheit(temp),
                              humidity,
                              celsius_to_fahrenheit(dew_point),
                              pascals_to_inches_mercury(bmp_pressure),
                              get_altitude(bmp_pressure)))
            sys.stdout.flush()
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        print ""
        GPIO.cleanup()


if __name__ == "__main__":
    main()
