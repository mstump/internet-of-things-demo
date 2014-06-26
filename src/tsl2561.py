#!/usr/bin/env python
import time, sys, ctypes, struct, math, smbus
import RPi.GPIO as GPIO

I2C_BUS                           = 1
TSL2561_ADDR                      = 0x39
TSL2561_REGISTER_CONTROL          = 0x00
TSL2561_REGISTER_TIMING           = 0x01
TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02
TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03
TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04
TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05
TSL2561_REGISTER_INTERRUPT        = 0x06
TSL2561_REGISTER_CRC              = 0x08
TSL2561_REGISTER_ID               = 0x0A
TSL2561_REGISTER_CHAN0_LOW        = 0x0C
TSL2561_REGISTER_CHAN0_HIGH       = 0x0D
TSL2561_REGISTER_CHAN1_LOW        = 0x0E
TSL2561_REGISTER_CHAN1_HIGH       = 0x0F
TSL2561_INTEGRATIONTIME_13MS      = 0x00    # 13.7ms
TSL2561_INTEGRATIONTIME_101MS     = 0x01    # 101ms
TSL2561_INTEGRATIONTIME_402MS     = 0x02    # 402ms
TSL2561_GAIN_1X                   = 0x00    # No gain
TSL2561_GAIN_16X                  = 0x10    # 16x gain
TSL2561_VISIBLE                   = 2       # channel 0 - channel 1
TSL2561_INFRARED                  = 1       # channel 1
TSL2561_FULLSPECTRUM              = 0       # channel 0
# I2C address options
TSL2561_ADDR_LOW                  = 0x29
TSL2561_ADDR_FLOAT                = 0x39    # Default address = pin left floating
TSL2561_ADDR_HIGH                 = 0x49
# Lux calculations differ slightly for CS package
TSL2561_COMMAND_BIT               = 0x80    # Must be 1
TSL2561_CLEAR_BIT                 = 0x40    # Clears any pending interrupt = write 1 to clear
TSL2561_WORD_BIT                  = 0x20    # 1 = read/write word = rather than byte
TSL2561_BLOCK_BIT                 = 0x10    # 1 = using block read/write
TSL2561_CONTROL_POWERON           = 0x03
TSL2561_CONTROL_POWEROFF          = 0x00
TSL2561_LUX_LUXSCALE              = 14      # Scale by 2^14
TSL2561_LUX_RATIOSCALE            = 9       # Scale ratio by 2^9
TSL2561_LUX_CHSCALE               = 10      # Scale channel values by 2^10
TSL2561_LUX_CHSCALE_TINT0         = 0x7517  # 322/11 * 2^TSL2561_LUX_CHSCALE
TSL2561_LUX_CHSCALE_TINT1         = 0x0FE7  # 322/81 * 2^TSL2561_LUX_CHSCALE
# T, FN and CL package values
TSL2561_LUX_K1T                   = 0x0040  # 0.125 * 2^RATIO_SCALE
TSL2561_LUX_B1T                   = 0x01f2  # 0.0304 * 2^LUX_SCALE
TSL2561_LUX_M1T                   = 0x01be  # 0.0272 * 2^LUX_SCALE
TSL2561_LUX_K2T                   = 0x0080  # 0.250 * 2^RATIO_SCALE
TSL2561_LUX_B2T                   = 0x0214  # 0.0325 * 2^LUX_SCALE
TSL2561_LUX_M2T                   = 0x02d1  # 0.0440 * 2^LUX_SCALE
TSL2561_LUX_K3T                   = 0x00c0  # 0.375 * 2^RATIO_SCALE
TSL2561_LUX_B3T                   = 0x023f  # 0.0351 * 2^LUX_SCALE
TSL2561_LUX_M3T                   = 0x037b  # 0.0544 * 2^LUX_SCALE
TSL2561_LUX_K4T                   = 0x0100  # 0.50 * 2^RATIO_SCALE
TSL2561_LUX_B4T                   = 0x0270  # 0.0381 * 2^LUX_SCALE
TSL2561_LUX_M4T                   = 0x03fe  # 0.0624 * 2^LUX_SCALE
TSL2561_LUX_K5T                   = 0x0138  # 0.61 * 2^RATIO_SCALE
TSL2561_LUX_B5T                   = 0x016f  # 0.0224 * 2^LUX_SCALE
TSL2561_LUX_M5T                   = 0x01fc  # 0.0310 * 2^LUX_SCALE
TSL2561_LUX_K6T                   = 0x019a  # 0.80 * 2^RATIO_SCALE
TSL2561_LUX_B6T                   = 0x00d2  # 0.0128 * 2^LUX_SCALE
TSL2561_LUX_M6T                   = 0x00fb  # 0.0153 * 2^LUX_SCALE
TSL2561_LUX_K7T                   = 0x029a  # 1.3 * 2^RATIO_SCALE
TSL2561_LUX_B7T                   = 0x0018  # 0.00146 * 2^LUX_SCALE
TSL2561_LUX_M7T                   = 0x0012  # 0.00112 * 2^LUX_SCALE
TSL2561_LUX_K8T                   = 0x029a  # 1.3 * 2^RATIO_SCALE
TSL2561_LUX_B8T                   = 0x0000  # 0.000 * 2^LUX_SCALE
TSL2561_LUX_M8T                   = 0x0000  # 0.000 * 2^LUX_SCALE
# Auto-gain thresholds
TSL2561_AGC_THI_13MS              = 4850    # Max value at Ti 13ms = 5047
TSL2561_AGC_TLO_13MS              = 100
TSL2561_AGC_THI_101MS             = 36000   # Max value at Ti 101ms = 37177
TSL2561_AGC_TLO_101MS             = 200
TSL2561_AGC_THI_402MS             = 63000   # Max value at Ti 402ms = 65535
TSL2561_AGC_TLO_402MS             = 500
# Clipping thresholds
TSL2561_CLIPPING_13MS             = 4900
TSL2561_CLIPPING_101MS            = 37000
TSL2561_CLIPPING_402MS            = 65000


usleep = lambda x: time.sleep(x / 1000000.0)


def ic2_read_u16(i2c, address, register):
    hi = i2c.read_byte_data(address, register)
    lo = i2c.read_byte_data(address, register + 1)
    return (hi << 8) + lo


def ic2_read_s8(i2c, address, register):
    result = i2c.read_byte_data(address, register)
    if result > 127:
        result -= 256
    return result


def ic2_read_s16(i2c, address, register):
    hi = ic2_read_s8(i2c, address, register)
    lo = i2c.read_byte_data(address, register + 1)
    return (hi << 8) + lo


def tsl2561_poweron(i2c, address):
    return i2c.write_byte_data(address, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON)


def tsl2561_poweroff(i2c, address):
    return i2c.write_byte_data(address, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF)


def tsl2561_setup(i2c, address, gain, integration_time):
    id = ic2_read_s8(i2c, address, TSL2561_REGISTER_ID)
    if not (id & TSL2561_REGISTER_ID):
        return False

    # turn the device on
    i2c.write_byte_data(address, TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, integration_time | gain)
    return True


def tsl2561_getdata(i2c, address, integration_time):
    if integration_time == TSL2561_INTEGRATIONTIME_13MS:
        usleep(14000)

    elif integration_time == TSL2561_INTEGRATIONTIME_101MS:
        usleep(102000)

    else:
        usleep(403000)

    # visible + infared
    broadband = ic2_read_u16(i2c, address, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW)

    # infared
    ir = ic2_read_u16(i2c, address, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW)
    tsl2561_poweroff(i2c, address)
    return broadband, ir


def calculate_lux(gain, integration_time, broadband, ir):
    # make sure the sensor isn't saturated
    clip_threshold = TSL2561_CLIPPING_402MS

    if integration_time == TSL2561_INTEGRATIONTIME_13MS:
        clip_threshold = TSL2561_CLIPPING_13MS

    elif integration_time == TSL2561_INTEGRATIONTIME_101MS:
        clip_threshold = TSL2561_CLIPPING_101MS

    # Return 0 lux if the sensor is saturated
    if ((broadband > clip_threshold) or (ir > clip_threshold)):
        return 0

    # Get the correct scale depending on the intergration time
    ch_scale = (1 << TSL2561_LUX_CHSCALE);
    if integration_time == TSL2561_INTEGRATIONTIME_13MS:
        ch_scale = TSL2561_LUX_CHSCALE_TINT0;

    elif integration_time == TSL2561_INTEGRATIONTIME_101MS:
        ch_scale = TSL2561_LUX_CHSCALE_TINT1;

    # Scale for gain (1x or 16x)
    if not gain:
        ch_scale = ch_scale << 4;

    # Scale the channel values
    channel0 = (broadband * ch_scale) >> TSL2561_LUX_CHSCALE
    channel1 = (ir * ch_scale) >> TSL2561_LUX_CHSCALE

    # Find the ratio of the channel values (Channel1/Channel0)
    ratio = 0;
    if (channel0 != 0):
        ratio = (((channel1 << (TSL2561_LUX_RATIOSCALE + 1)) / channel0) + 1) >> 1

    b = 0
    m = 0

    if ((ratio >= 0) and (ratio <= TSL2561_LUX_K1T)):
        b = TSL2561_LUX_B1T
        m = TSL2561_LUX_M1T

    elif (ratio <= TSL2561_LUX_K2T):
        b = TSL2561_LUX_B2T
        m = TSL2561_LUX_M2T

    elif (ratio <= TSL2561_LUX_K3T):
        b = TSL2561_LUX_B3T
        m = TSL2561_LUX_M3T

    elif (ratio <= TSL2561_LUX_K4T):
        b = TSL2561_LUX_B4T
        m = TSL2561_LUX_M4T

    elif (ratio <= TSL2561_LUX_K5T):
        b = TSL2561_LUX_B5T
        m = TSL2561_LUX_M5T

    elif (ratio <= TSL2561_LUX_K6T):
        b = TSL2561_LUX_B6T
        m = TSL2561_LUX_M6T

    elif (ratio <= TSL2561_LUX_K7T):
        b = TSL2561_LUX_B7T
        m = TSL2561_LUX_M7T

    elif (ratio > TSL2561_LUX_K8T):
        b = TSL2561_LUX_B8T
        m = TSL2561_LUX_M8T

    temp = ((channel0 * b) - (channel1 * m))

    # Do not allow negative lux value
    if (temp < 0):
        temp = 0

    # Round lsb (2^(LUX_SCALE-1))
    temp += (1 << (TSL2561_LUX_LUXSCALE - 1))

    # Strip off fractional portion
    lux = temp >> TSL2561_LUX_LUXSCALE

    # Signal I2C had no errors
    return lux;


def main():
    gain = TSL2561_GAIN_16X
    integration_time = TSL2561_INTEGRATIONTIME_402MS
    address = TSL2561_ADDR

    i2c = smbus.SMBus(I2C_BUS)
    GPIO.setmode(GPIO.BCM)

    tsl2561_poweron(i2c, address)
    time.sleep(1)
    if not tsl2561_setup(i2c, address, gain, integration_time):
        tsl2561_poweroff(i2c, address)
        print "ERROR"
        sys.exit(1)

    broadband, ir = tsl2561_getdata(i2c, address, integration_time)
    lux = calculate_lux(gain, integration_time, broadband, ir)
    tsl2561_poweroff(i2c, address)
    print repr([broadband, ir, lux])


if __name__ == "__main__":
    main()
