#!/usr/bin/env python

import rospy
from roboclaw.roboclaw import RoboClaw
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

# TODO: Convert node to use msgs of type Twist.msg, and use teleop_twist_joy as a
# interface

class XboxDriver(object):
    DRIVE_SPEED = 10000
    DRIVE_STEER = 1000
    DRIVE_ACCEL = 100

    # RoboClaw Settings
    MIN_VOLTAGE = 10.5  # Volts
    MAX_VOLTAGE = 14.0  # Volts
    MAX_CURRENT_STEER = 10.0  # Amps
    MAX_CURRENT_DRIVE = 15.0  # Amps

    STEER_VEL_P = 0.1
    STEER_VEL_I = 0
    STEER_VEL_D = 0
    STEER_QPPR = 7000

    STEER_POS_P = 15
    STEER_POS_I = 0
    STEER_POS_D = 0
    STEER_POS_DEADZONE = 10
    STEER_POS_MAX = DRIVE_STEER
    STEER_POS_MIN = -STEER_POS_MAX

    #values arrived at from ion studio calibration
    DRIVE_VEL_P = 10000
    DRIVE_VEL_I = 400
    DRIVE_VEL_D = 0
    DRIVE_QPPR = 6630

    def __init__(self):
        rospy.init_node('xbox_driver')
        rospy.on_shutdown(self._shutdown_callback)

        self._roboclaws = {
            'front': RoboClaw("/dev/front", 115200),
            'rear': RoboClaw("/dev/rear", 115200),
        }

        for claw in self._roboclaws.values():
            # Setup General
            claw.SetMainVoltages(
                int(self.MIN_VOLTAGE * 10),
                int(self.MAX_VOLTAGE * 10))
            claw.SetLogicVoltages(
                int(self.MIN_VOLTAGE * 10),
                int(self.MAX_VOLTAGE * 10))
            claw.SetEncM2(0)  # Only reset drive encoder
            # Setup Steer
            claw.SetM1MaxCurrent(int(self.MAX_CURRENT_STEER * 100))
            claw.SetM1VelocityPID(
                self.STEER_VEL_P,
                self.STEER_VEL_I,
                self.STEER_VEL_D,
                self.STEER_QPPR)
            claw.SetM1PositionPID(
                self.STEER_POS_P,
                self.STEER_POS_I,
                self.STEER_POS_D,
                0,
                self.STEER_POS_DEADZONE,
                self.STEER_POS_MIN,
                self.STEER_POS_MAX)
            # Setup Drive
            claw.SetM2MaxCurrent(int(self.MAX_CURRENT_DRIVE * 100))
            claw.SetM2VelocityPID(
                self.DRIVE_VEL_P,
                self.DRIVE_VEL_I,
                self.DRIVE_VEL_D,
                self.DRIVE_QPPR)

            claw.SetM2DefaultAccel(self.DRIVE_ACCEL)

        self.safe = True
        rospy.Subscriber('/ranger/brinkmanship/go', Bool, self._safeguard_callback)
        rospy.Subscriber('/joy', Joy, self._drive_callback)

        self.drive_enabled = False
        self.safeguarded = False
        rospy.Timer(rospy.Duration(0.5), self._timer_callback)

        rospy.loginfo("XboxDriver: Started")
        rospy.spin()

    def _timer_callback(self, event):
        if self.safeguarded == True:
            self.safeguarded = False
        else:
            self.drive_enabled = False
            rospy.logerr('Safeguarding offline! Drive disabled!')
            for claw in self._roboclaws.values():
                claw.SpeedAccelDeccelPositionM1(0, 0, 0, 0, 1)
                claw.SpeedM2(0)

    def _safeguard_callback(self, msg):
        if self.drive_enabled == False:
            self.drive_enabled = True
            rospy.logwarn('Safeguarding online! Drive enabled!')
        self.safeguarded = True
        
        if msg.data == True and self.safe == False:
            rospy.logwarn('Safe to drive!')
            self.safe = True
        elif msg.data == False and self.safe == True:
            rospy.logwarn('Untraversable terrain ahead! Stop!')
            self.safe = False
            for claw in self._roboclaws.values():
                claw.SpeedAccelDeccelPositionM1(0, 0, 0, 0, 1)
                claw.SpeedM2(0)

    def _drive_callback(self, msg):
        claw_front = self._roboclaws['front']
        claw_rear = self._roboclaws['rear']

        speed_value = self.DRIVE_SPEED * msg.axes[1];
        steer_value = self.DRIVE_STEER * msg.axes[3];
 
        if self.drive_enabled == True and (self.safe == True or speed_value < 0):
            claw_front.SpeedAccelDeccelPositionM1(0, 0, 0, int(steer_value), 1)
            claw_rear.SpeedAccelDeccelPositionM1(0, 0, 0, -int(steer_value), 1)
            claw_front.SpeedM2(int(speed_value))
            claw_rear.SpeedM2(int(speed_value)) 

    def _shutdown_callback(self):
        rospy.logwarn("Killing RoboClawDriver")
        for claw in self._roboclaws.values():
            claw.SpeedAccelDeccelPositionM1(0, 0, 0, 0, 1)
            claw.SpeedM2(0)
            claw.kill()

    def getGPIOValue(self, n):
        f = RoboClawDriver.files[n]
        value = f.read()
        value = value.replace('\n','').replace(' ','')
        f.seek(0)
        return int(value)

if __name__ == '__main__':
    try:
        xd = XboxDriver()
    except rospy.ROSInterruptException:
        pass
