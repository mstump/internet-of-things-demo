#!/usr/bin/env python
import time, sys, ctypes, struct, math, smbus
import RPi.GPIO as GPIO

I2C_BUS           = 1
ADS1115_ADDRESS   = 0x48
ADS1115_ALERT_PIN = 13

ADS1115_REG_POINTER_CONVERT = 0x00
ADS1115_REG_POINTER_CONFIG  = 0x01

ADS1115_REG_CONFIG_OS_SINGLE     = 0x8000
ADS1115_REG_CONFIG_PGA_6_144V    = 0x0000
ADS1015_REG_CONFIG_PGA_1_024V    = 0x0600
ADS1015_REG_CONFIG_PGA_0_512V    = 0x0800
ADS1015_REG_CONFIG_PGA_0_256V    = 0x0A00
ADS1115_REG_CONFIG_MUX_DIFF_2_3  = 0x3000
ADS1115_REG_CONFIG_CLAT_NONLAT   = 0x0000
ADS1115_REG_CONFIG_CQUE_NONE     = 0x0003
ADS1115_REG_CONFIG_CPOL_ACTVLOW  = 0x0000
ADS1115_REG_CONFIG_CPOL_ACTVHIGH = 0x0008
ADS1115_REG_CONFIG_CMODE_TRAD    = 0x0000
ADS1115_REG_CONFIG_DR_0250SPS    = 0x0020
ADS1115_REG_CONFIG_DR_1600SPS    = 0x0080
ADS1115_REG_CONFIG_MODE_CONTIN   = 0x0000
ADS1115_REG_CONFIG_MODE_SINGLE   = 0x0100


LOAD_A = 646 # grams
MV_A   = -0.6953125
LOAD_B = 129
MV_B   = -0.25
TARE   = 2.01754385965 # the weight of the scale platform

usleep = lambda x: time.sleep(x / 1000000.0)


def map_float(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


def ads1115_get_differential_23(i2c, address, alert_pin):
    config = ADS1115_REG_CONFIG_CQUE_NONE \
        | ADS1115_REG_CONFIG_CLAT_NONLAT  \
        | ADS1115_REG_CONFIG_CPOL_ACTVLOW \
        | ADS1115_REG_CONFIG_CMODE_TRAD   \
        | ADS1115_REG_CONFIG_DR_0250SPS   \
        | ADS1115_REG_CONFIG_MODE_SINGLE

    # gain
    config |= ADS1015_REG_CONFIG_PGA_0_256V

    # Set channels
    config |= ADS1115_REG_CONFIG_MUX_DIFF_2_3

    # Set 'start single-conversion' bit
    config |= ADS1115_REG_CONFIG_OS_SINGLE

    bytes = [(config >> 8) & 0xFF, config & 0xFF]

    # Write config register to the ADC
    i2c.write_i2c_block_data(address, ADS1115_REG_POINTER_CONFIG, bytes)

    while GPIO.input(alert_pin):
        usleep(100)

    result = i2c.read_i2c_block_data(address, ADS1115_REG_POINTER_CONVERT, 2)

    val = (result[0] << 8) | (result[1])
    if val > 0x7FFF:
        return (val - 0xFFFF) * 256 / 32768.0
    else:
        return ( (result[0] << 8) | (result[1]) ) * 256 / 32768.0


def ads1115_setup(i2c, address, alert_pin):
    GPIO.setup(ADS1115_ALERT_PIN, GPIO.IN)


def main():
    i2c       = smbus.SMBus(I2C_BUS)
    address   = ADS1115_ADDRESS
    alert_pin = ADS1115_ALERT_PIN

    try:
        GPIO.setmode(GPIO.BCM)
        ads1115_setup(i2c, address, alert_pin)
        mv = ads1115_get_differential_23(i2c, address, alert_pin)
        grams = map_float(mv, MV_A, MV_B, LOAD_A, LOAD_B);

        print mv, "mV"
        print int(grams - TARE), "grams"


    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
