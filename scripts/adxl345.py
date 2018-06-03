#!/usr/bin/env python
#encoding: utf8

import RPi.GPIO as GPIO
import os
import smbus
import time
import math


class ADXL345():

    def __init__(self):
        self.dev_addr = 0x1d
        my_bus = ""

        if GPIO.RPI_INFO['P1_REVISION'] == 1:
            my_bus = 0
        else:
            my_bus = 1

        self.sm_bus = smbus.SMBus(my_bus)

        self._prev_value = {'x':0, 'y':0, 'z':0}
        self._addr = {'x':0x32, 'y':0x34, 'z':0x36}
        self._axes = ['x', 'y', 'z']


    def setUp(self):
        self.sm_bus.write_byte_data(self.dev_addr, 0x2c, 0x0b) # bandwidth rate 200 Hz
        self.sm_bus.write_byte_data(self.dev_addr, 0x31, 0x00) # DATA_FORMAT 10bit 2g
        self.sm_bus.write_byte_data(self.dev_addr, 0x38, 0x00) # FIFO_CTL OFF
        self.sm_bus.write_byte_data(self.dev_addr, 0x2d, 0x08) # POWER_CTL Enable

    def _get_value(self, axis):
        addr = self._addr[axis]

        try:
            # データは2の補数で表現されている
            hi_byte = self.sm_bus.read_byte_data(self.dev_addr, addr+1)
            lo_byte = self.sm_bus.read_byte_data(self.dev_addr, addr)
            full_byte = hi_byte << 8 | lo_byte

            signed_value = -(full_byte & 0x8000) | (full_byte & 0x7FFF)

            # 外れ値を除去。外れ値の発生原因は未調査
            if self._is_outlier(axis, signed_value):
                return self._prev_value[axis]

            self._prev_value[axis] = signed_value
            return signed_value
        except IOError:
            # i2cバスのケーブルが抜けるとIOErrorが発生するので、
            # その場合は前回の値を出力する
            return self._prev_value[axis]


    def get_value_x(self):
        return self._get_value('x')

    def get_value_y(self):
        return self._get_value('y')

    def get_value_z(self):
        return self._get_value('z')

    def get_value(self, axis):
        if axis in self._axes:
            return self._get_value(axis)
        else:
            return 0

    def _is_outlier(self, axis, current_value):
        diff = math.fabs(self._prev_value[axis] - current_value)

        # 値がしきい値より大きく変動したら外れ値とみなす。
        # 外れ値の発生原因は未調査
        if diff > 200:
            return True
        else:
            return False



if __name__ == '__main__':
    adxl = ADXL345()
    adxl.setUp()

    for i in range(1000):
        x = adxl.get_value_x()
        y = adxl.get_value_y()
        z = adxl.get_value_z()

        os.system("clear")
        print("X=", x)
        print("Y=", y)
        print("Z=", z)
        time.sleep(0.1)
