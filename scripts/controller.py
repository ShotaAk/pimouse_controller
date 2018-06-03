#!/usr/bin/env python
#encoding: utf8

import rospy
import math
import RPi.GPIO as GPIO

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

from adxl345 import ADXL345
from average_filter import AverageFilter

class AccelConverter():
    def __init__(self, acc, gain, max_vel, halt_acc, halt_vel):
        self._target_vel = 0        # 速度出力値
        self._ACC = acc             # 加速度
        self._VEL_GAIN = gain       # 加速度センサの値を目標速度に変換するゲイン
        self._MAX_VEL = max_vel     # 最大出力速度
        self._HALT_ACC = halt_acc   # 入力が0になったと見なす加速度センサしきい値
        self._HALT_VEL = halt_vel   # 速度が0になったと見なす速度しきい値

    def convert(self, accel):
        velocity = 0
        if math.fabs(accel) > self._HALT_ACC:
            velocity = accel * self._VEL_GAIN
            
            # velocity limitter
            if math.fabs(velocity) > self._MAX_VEL:
                velocity = math.copysign(self._MAX_VEL, velocity)

            self._accelerate(velocity)
        else:
            self._accelerate(velocity)

            if math.fabs(self._target_vel) < self._HALT_VEL:
                self._target_vel = 0

    def _accelerate(self, velocity):
        if velocity > self._target_vel:
            self._target_vel += self._ACC
        elif velocity < self._target_vel:
            self._target_vel -= self._ACC

    def reset(self):
        self._target_vel = 0

    def target_velocity(self):
        return self._target_vel



class Sender():
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self._converter_forward = AccelConverter(
                acc = 0.01,         # unit: m/s^2
                gain = 1.0/100.0,   
                max_vel = 2.0,      # unit: m/s
                halt_acc = 50,      # accel sensor value, unit: rad/s^2
                halt_vel = 0.02     # cmd_vel value, unit: m/s
                )
        self._converter_angular = AccelConverter(
                acc = 0.05,         # unit: rad/s^2
                gain = 1.0/100.0,
                max_vel = 2.0,      # unit: rad/s
                halt_acc = 50,      # accel sensor value, unit: rad/s^2
                halt_vel = 0.02     # cmd_vel value, unit: rad/s
                )

    def convert_accel(self, accel_x, accel_y):
        self._converter_forward.convert(-accel_y)
        self._converter_angular.convert(-accel_x)

    def halt(self):
        self._converter_forward.reset()
        self._converter_angular.reset()

    def send(self):
        twist = Twist()
        twist.linear.x = self._converter_forward.target_velocity()
        twist.angular.z = self._converter_angular.target_velocity()
        self._cmd_vel_pub.publish(twist)


def main():
    # GPIO Settings
    PIN_RESTART = 4 # GPIO PIN for motor_on and filter offset
    PIN_STOP = 17   # GPIO PIN for motor_off
    SW_ON = GPIO.LOW # This value assignd by circuit logic
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_RESTART, GPIO.IN)
    GPIO.setup(PIN_STOP, GPIO.IN)

    # Module Settings
    sender = Sender()
    adxl = ADXL345()
    adxl.setUp()

    avr_filter = {}
    avr_filter['x'] = AverageFilter(10)
    avr_filter['y'] = AverageFilter(10)
    avr_filter['z'] = AverageFilter(10)

    # Publisher Settings
    accel_pubs = {}
    accel_pubs['x'] = rospy.Publisher('accel_x', Float32, queue_size=10)
    accel_pubs['y'] = rospy.Publisher('accel_y', Float32, queue_size=10)
    accel_pubs['z'] = rospy.Publisher('accel_z', Float32, queue_size=10)

    raw_accel_pubs = {}
    raw_accel_pubs['x'] = rospy.Publisher('raw_accel_x', Float32, queue_size=10)
    raw_accel_pubs['y'] = rospy.Publisher('raw_accel_y', Float32, queue_size=10)
    raw_accel_pubs['z'] = rospy.Publisher('raw_accel_z', Float32, queue_size=10)

    # Service Settings
    motor_on = rospy.ServiceProxy('/motor_on', Trigger)
    motor_off = rospy.ServiceProxy('/motor_off', Trigger)

    # Paramter Settings
    axes = ['x', 'y', 'z']
    accel = {'x':0, 'y':0, 'z':0}
    can_send = False

    rospy.init_node('pimouse_controller')
    r = rospy.Rate(60)

    while not rospy.is_shutdown():
        # restart
        if GPIO.input(PIN_RESTART) == SW_ON:
            # motor_on
            result = motor_on()
            if result.success:
                rospy.logdebug('motor restart')
                can_send = True
            else:
                rospy.logerr('can not restart motor')

            # offset accel
            for axis in axes:
                avr_filter[axis].offset()

        # stop
        if GPIO.input(PIN_STOP) == SW_ON:
            # motor_off
            result = motor_off()
            can_send = False # for safety, turn off without service responce 
            if result.success:
                rospy.logdebug('motor stop')
            else:
                rospy.logerr('can not stop motor')

            # halt command
            sender.halt()

        # receive value
        for axis in axes:
            raw_value = adxl.get_value(axis)

            avr_filter[axis].update(raw_value)
            accel[axis] = avr_filter[axis].get_value()

            accel_pubs[axis].publish(accel[axis])
            raw_accel_pubs[axis].publish(raw_value)

        # send command
        if can_send:
            sender.convert_accel(accel['x'], accel['y'])
            sender.send()
        r.sleep()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException: pass

