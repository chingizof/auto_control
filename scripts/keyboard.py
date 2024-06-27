#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard

class KeyboardController:
    def __init__(self):
        self.pub = rospy.Publisher('keyboard_control', String, queue_size=10)
        rospy.init_node('keyboard_controller', anonymous=True)
        self.rate = rospy.Rate(10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.pub.publish('A')
            elif key == keyboard.Key.down:
                self.pub.publish('B')
            elif key == keyboard.Key.left:
                self.pub.publish('C')
            elif key == keyboard.Key.right:
                self.pub.publish('D')
        except AttributeError:
            pass

    def run(self):
        rospy.loginfo("Keyboard controller started. Press arrow keys to send commands.")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
