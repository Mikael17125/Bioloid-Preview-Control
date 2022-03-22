#!/usr/bin/env python3

import smbus
from math import atan2, sqrt, pi
from time import sleep


class MPU:

    def __init__(self):

        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
        self.addr = 0x68

        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(self.addr, self.power_mgmt_1, 0)

    def get_gyro_data(self):
        gx = self.read_word_2c(0x43) * pi / (180.0 * 131.0)
        gy = self.read_word_2c(0x45) * pi / (180.0 * 131.0)
        gz = self.read_word_2c(0x47) * pi / (180.0 * 131.0)
        return {'x': gx, 'y': gy, 'z': gz}

        # m/s^2
    def get_accel_data(self):
        ax = self.read_word_2c(0x3b) / 16384.0
        ay = self.read_word_2c(0x3d) / 16384.0
        az = self.read_word_2c(0x3f) / 16384.0
        return {'x': ax, 'y': ay, 'z': az}

    def get_angle_data(self):
        angle = [.0, .0, .0]
        accel = self.get_accel_data()

        angle[0] = atan2(accel['y'], sqrt(accel['x'] ** 2.0 + accel['z'] ** 2.0))
        angle[1] = atan2(-accel['x'], sqrt(accel['y'] ** 2.0 + accel['z'] ** 2.0))
        angle[2] = 0

        return {'x': angle[0], 'y': angle[1], 'z': angle[2]}

    def read_word(self, reg_adr):
        high = self.bus.read_byte_data(self.addr, reg_adr)
        low = self.bus.read_byte_data(self.addr, reg_adr + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, reg_adr):
        val = self.read_word(reg_adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

