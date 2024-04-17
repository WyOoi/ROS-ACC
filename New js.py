#!/usr/bin/python

import rospy
import threading
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.condition = threading.Condition()
        self.done = False
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        self.start()

    def run(self):
        while not self.done:
            with self.condition:
                if self.condition.wait(self.timeout):
                    break
            self.publisher.publish(Twist())

class RaspberryPiPWMInput():
    def __init__(self):
        rospy.init_node('teleop_js')
        self.repeat = 10  # Publish rate in Hz
        self.pub_thread = PublishThread(self.repeat)

        # Define GPIO pin numbers
        self.CH_THROTTLE_IN = 26  # Replace with your desired PWM input pin (BCM numbering)
        self.CH_STEERING_IN = 20  # Replace with your desired PWM input pin (BCM numbering)
        self.PWM_DISCONNECT_THRESHOLD = 700  # Microsecond threshold for disconnection

        GPIO.setmode(GPIO.BCM)

        # Set up input channels as inputs
        for pin in [self.CH_THROTTLE_IN, self.CH_STEERING_IN]:
            GPIO.setup(pin, GPIO.IN)

        self._pwm_min = 900
        self._pwm_max = 2100
        self._offset_throttle = 0
        self._offset_steering = 0
        self._values_received = 0

        self.ros_twist_msg = Twist()

        self._timeout_s = 2
        self._last_time_cmd_rcv = time.time() - self._timeout_s

    def measure_pulse_width(self, channel):
        GPIO.wait_for_edge(channel, GPIO.RISING)  # Wait for rising edge
        start_time = time.time() * 1e6  # Convert to microseconds
        GPIO.wait_for_edge(channel, GPIO.FALLING)  # Wait for falling edge
        end_time = time.time() * 1e6  # Convert to microseconds
        pulse_width = end_time - start_time
        return pulse_width

    def read_pwm_values(self):
        channel_values = [self.measure_pulse_width(self.CH_THROTTLE_IN), self.measure_pulse_width(self.CH_STEERING_IN)]
        disconnection = any(value < self.PWM_DISCONNECT_THRESHOLD for value in channel_values)
        return channel_values, disconnection

    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm

    @property
    def is_rc_connected(self):
        return self._values_received >= 2

    def run(self):
        rate = rospy.Rate(self.repeat)
        while not rospy.is_shutdown():
            # Read PWM values from Raspberry Pi GPIO pins
            channel_values, is_disconnected = self.read_pwm_values()

            # Convert PWM values to Twist messages
            self.ros_twist_msg.linear.x = self.pwm_to_adimensional(channel_values[0] + self._offset_throttle)
            self.ros_twist_msg.angular.z = self.pwm_to_adimensional(channel_values[1] + self._offset_steering)

            # Publish Twist message
            self.pub_thread.publisher.publish(self.ros_twist_msg)

            # Sleep or add rate control here if needed
            rate.sleep()

if __name__ == "__main__":
    try:
        pi_pwm_input = RaspberryPiPWMInput()
        pi_pwm_input.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()  # Clean up GPIO resources
