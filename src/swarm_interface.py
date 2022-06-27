#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

#is it controlled now, let's go, hm
class SwarmInterface(object):
    def __init__(self):
        rospy.init_node('swarm_interface')
        self.test_pub = rospy.Publisher('/test_topic', String, queue_size=10)
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.test_pub.publish("test")
            rate.sleep()

if __name__ == "__main__":
    node = SwarmInterface()
    node.run()