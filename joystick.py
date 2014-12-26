#!/usr/bin/env python
"""
Send joystick data over serial port to my home made Syma S107G helicopter radio
controller (STM32 based). Serial port data from the STM32 is printed on stdout.

Tested with E-sky RC-radio joystick.
"""

import struct
import serial
import sys
import time
import threading


def clip(val, minval, maxval):
    if val > maxval:
        val = maxval
    elif val < minval:
        val = minval
    return val


class Joystick:
    def __init__(self, device="/dev/input/js0"):
        self.jsfile = open(device, "r")
        self.a1x = 0
        self.a1y = 0
        self.a2x = 0
        self.a2y = 0
        # TODO: get the initial joystick state
        self.lock = threading.Lock()
        self.th = threading.Thread(target=self._read_loop)
        self.th.daemon = True
        self.th.start()

    def read(self):
        buf = self.jsfile.read(8)
        if len(buf) != 8:
            raise Exception("read less than 8 bytes from joystick device file")

        # What we read from the joystick device file is a (series of) struct
        # input_event structures. They are defined in
        # <linux>/include/uapi/linux/input.h or /usr/include/input.h
        (ev_time, ev_value, ev_type, ev_number) = struct.unpack("=LhBB", buf)

        # scale down to 0 - 255
        with self.lock:
            if ev_number == 0:
                self.a2x = (ev_value / 180) + 128
            if ev_number == 1:
                self.a2y = (ev_value / 180) + 128
            if ev_number == 2:
                self.a1y = (ev_value / 180) + 128
            if ev_number == 4:
                self.a1x = (ev_value / 180) + 128

            # limit axes
            self.a1x = clip(self.a1x, 0, 255)
            self.a1y = clip(self.a1y, 0, 255)
            self.a2x = clip(self.a2x, 0, 255)
            self.a2y = clip(self.a2y, 0, 255)

    def _read_loop(self):
        while True:
            self.read()

    def get_axes(self):
        with self.lock:
            values = (self.a1x, self.a1y, self.a2x, self.a2y)
        return values


def main():
    js = Joystick("/dev/input/js0")
    # timeout=0 means non-blocking serial port
    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0)
    tnow = 0
    while True:
        if time.time() >= (tnow + 0.1):
            ax = js.get_axes()
            # print them in the order that my S107 Heli control program expects it
            ser.write("%d %d %d %d\r" % (ax[0], ax[3], ax[1], ax[2]))
            tnow = time.time()
        line = ser.readline()
        if line:
            #print("%f %s" % (time.time(), line))
            sys.stdout.write(line)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt as e:
        print("Interrupted with Ctrl-C. Exiting.")
        pass
