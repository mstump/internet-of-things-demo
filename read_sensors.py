import time, sys, ctypes, struct, curses
import RPi.GPIO as GPIO

MOTION_SENSOR_PIN = 4
LED_PIN = 18

def setup():
    GPIO.setmode(GPIO.BCM)
    setup_motion_sensor(MOTION_SENSOR_PIN)

def setup_motion_sensor(input_pin):
    GPIO.setup(input_pin, GPIO.IN)

def motion_sensor(pin):
    return GPIO.input(pin)


def shift_out(data_pin, clock_pin, value, lsb_first=True, bits=8, delay_micros=0):
    data = []

    if type(value) == type(1):
        data = struct.pack('i', value)

    elif type(value) == type('a'):
        data = [value]

    elif type(value) == type("a"):
        data = value

    else:
        raise Exception("shift_out unsupported type: type(value) => %s" % type(value))

    for i in data:
        shift_out_byte(data_pin, clock_pin, i, lsb_first, bits, delay_micros)


def shift_out_byte(data_pin, clock_pin, value, lsb_first=True, bits=8, delay_micros=0):
    for i in xrange(0, bits):
        if lsb_first:
            GPIO.output(data_pin, bool(value & (1 << i)))
        else:
            GPIO.output(data_pin, bool(value & (1 << ((bits - 1 - i )))))

        GPIO.output(clock_pin, GPIO.HIGH)
        time.usleep(delay_micros)
        GPIO.output(clock_pin, GPIO.LOW)


def sht_send_command(data_pin, clock_pin, command):
    GPIO.setup(data_pin, GPIO.OUT)
    GPIO.setup(clock_pin, GPIO.OUT)

    GPIO.output(data_pin, GPIO.HIGH)
    GPIO.output(clock_pin, GPIO.HIGH)

    GPIO.output(data_pin, GPIO.LOW)
    GPIO.output(clock_pin, GPIO.LOW)

    GPIO.output(clock_pin, GPIO.HIGH)
    GPIO.output(data_pin, GPIO.HIGH)
    GPIO.output(clock_pin, GPIO.LOW)
    shift_out(data_pin, clock_pin, False, command)

    GPIO.setup(data_pin, GPIO.IN)
    GPIO.output(clock_pin, GPIO.HIGH)
    if GPIO.input(pin):
        print "ACK error 0"

    GPIO.output(clock_pin, GPIO.LOW)
    if GPIO.input(pin):
        print "ACK error 1"


def sht_wait_result(data_pin):
    GPIO.setup(data_pin, GPIO.IN)

    # wait for the SHTx answer
    # need to wait up to 2 seconds for the value
    ack = True
    for i in xrange(0, 1000):
        time.usleep(2)
        ack = GPIO.input(data_pin)
        if ack == False:
            break

    if ack == True:
        print "ACK error 2"


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
    GPIO.setup(data_pin, INPUT)
    LSB = shift_in_byte(data_pin, clock_pin, False)
    return ((MSB << 8) | LSB)  # combine bits


def sht_skip_crc(data_pin, clock_pin):
    # skip CRC data from the SHTx sensor
    GPIO.setup(data_pin, GPIO.OUT)
    GPIO.setup(clock_pin, GPIO.OUT)
    GPIO.output(data_pin, GPIO.HIGH)
    GPIO.output(clock_pin, GPIO.HIGH)
    GPIO.output(clock_pin, GPIO.LOW)


def sht_get_temp(data_pin, clock_pin):
  # temperature in celsius
  sht_send_command(B00000011, data_pin, clock_pin)
  sht_wait_result(data_pin)

  val = sht_get_data(data_pin, clock_pin)
  sht_skip_crc(data_pin, clock_pin)
  return val * 0.01 - 40 # convert to celsius


def sht_get_humidity(data_pin, clock_pin):
  # relative Humidity
  sht_send_command(B00000101, data_pin, clock_pin)
  sht_wait_result(data_pin)

  val = sht_get_data(data_pin, clock_pin)
  sht_skip_crc(data_pin, clock_pin)
  return -4.0 + 0.0405 * val + -0.0000028 * val * val


def main():
    try:
        stdscr = curses.initscr()

        while True:
            motion   = motion_sensor(MOTION_SENSOR_PIN);
            temp     = sht_get_temp(SHT_DATA_PIN, SHT_CLOCK_PIN)
            humidity = sht_get_humidity(SHT_DATA_PIN, SHT_CLOCK_PIN)

            stdscr.addstr(0, 0, "motion: %s | temp: %s°C | relative humidity: %s" % (motion, temp, humidity))
            stdscr.refresh()
            time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
