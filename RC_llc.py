#!/usr/bin/python

"""
Class for low level control of owr car. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
import time


class ServoConvert():
    def __init__(self, id=1, center_value=90, range=180, direction=1):
        self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range
        self._dir       = direction
        self.id         = id

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        self.value_out  = max(int(self._dir*value_in*self._half_range + self._center), 0)
        print self.id, self.value_out
        return(self.value_out)

class DkLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('dk_llc')

        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=1)
        self.actuators['steering']  = ServoConvert(id=2, direction=1) #-- positive left
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg = []
        for i in range(2): 
            self._servo_msg.append(UInt16())

        #--- Create the servo array publisher
        self.ros_pub_servo_throttle   = rospy.Publisher("/servo_throttle", UInt16, queue_size=1)
        self.ros_pub_servo_steering   = rospy.Publisher("/servo_steering", UInt16, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber corrrectly initialized")

        #--- Get the last time e got a commands
        self._timeout_s             = 5
        self._last_time_cmd_rcv     = time.time() - self._timeout_s

        rospy.loginfo("Initialization complete")

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        self._servo_msg[0].data = self.actuators['throttle'].value_out
        self.ros_pub_servo_throttle.publish(self._servo_msg[0])
        self._servo_msg[1].data = self.actuators['steering'].value_out
        self.ros_pub_servo_steering.publish(self._servo_msg[1])        

    @property
    def is_controller_connected(self):
        print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print self._last_time_cmd_rcv, self.is_controller_connected
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    dk_llc     = DkLowLevelCtrl()
    dk_llc.run()
