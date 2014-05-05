import time, sys, ctypes, struct
# import RPi.GPIO as GPIO

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



def main():
    try:
        while True:
            motion = motion_sensor(MOTION_SENSOR_PIN);

    except KeyboardInterrupt:
        GPIO.cleanup()

# GPIO.cleanup()

shift_out(1, 1, 7)
