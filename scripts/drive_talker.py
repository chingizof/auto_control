#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pynput import keyboard

class DriveController:
    def __init__(self):
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
        rospy.init_node('drive_controller', anonymous=True)
        self.rate = rospy.Rate(1000) # 1000 Hz

        self.speed = 0.0
        self.steering_angle = 0.0

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.speed = 5.0
            elif key == keyboard.Key.down:
                self.speed = -5.0
            elif key == keyboard.Key.left:
                self.steering_angle = 0.34
            elif key == keyboard.Key.right:
                self.steering_angle = -0.34
            elif key == keyboard.Key.space:
                self.steering_angle = 0.0
                self.speed = 0.0
        except AttributeError:
            pass

    def run(self):
        rospy.loginfo("Drive controller started. Use arrow keys to control the vehicle.")
        while not rospy.is_shutdown():
            drive_cmd = AckermannDriveStamped()
            drive_cmd.drive.speed = self.speed
            drive_cmd.drive.steering_angle = self.steering_angle
            drive_cmd.drive.acceleration = 0.1
            drive_cmd.drive.jerk = 0.0

            #rospy.loginfo(f"Publishing: speed={self.speed}, steering_angle={self.steering_angle}")
            self.pub.publish(drive_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DriveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


