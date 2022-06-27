#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import tf
import numpy as np

#is it controlled now, let's go, hm
class SwarmInterface(object):
    def __init__(self):
        rospy.init_node('swarm_interface')
        self.listener = tf.TransformListener()
        self.test_pub = rospy.Publisher('/test_topic', String, queue_size=10)
        self.vel_pub = rospy.Publisher('/turtle4/cmd_vel',Twist,queue_size=10)
        self.edges = .20
        velMag = 0.2
        self.vel = np.array([np.random.uniform(),np.random.uniform()])
        self.vel = self.vel/np.linalg.norm(self.vel)
        self.vel *= velMag
        # self.wasLastInBounds = True
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                (trans,rot) =  self.listener.lookupTransform('/world','/vicon/turtle4/turtle4', rospy.Time(0))
            except:
                continue
            
            if self.edges<trans[0]:
                rospy.loginfo("Out of Bounds X Positive")
                if self.vel[0]>0:self.vel[0]*=-1 
            if -self.edges>trans[0]:
                rospy.loginfo("Out of Bounds X Negative")
                if self.vel[0]<0:self.vel[0]*=-1 
            if self.edges<trans[1]:
                rospy.loginfo("Out of Bounds Y Positive")
                if self.vel[1]>0:self.vel[1]*=-1 
            if -self.edges>trans[1]:
                rospy.loginfo("Out of Bounds Y Negative")
                if self.vel[1]<0:self.vel[1]*=-1 
            else:
                self.wasLastInBounds = True
                rospy.loginfo("In Bounds")
            rospy.loginfo("Current vel" + str(self.vel[0]) +" "+ str(self.vel[1]))

            vel_msg = Twist()
            vel_msg.linear.x = self.vel[0]
            vel_msg.linear.y = self.vel[1]
            self.vel_pub.publish(vel_msg)
            self.test_pub.publish("test "+str(trans[0])+" "+str(trans[1])+" "+str(trans[2]))
            rate.sleep()
    #     vel_msg = Twist()
    #     vel_msg.linear.x = 0
    #     vel_msg.linear.y = 0
    #     self.vel_pub.publish(vel_msg)
    # def on_shutdown(self):
    #     vel_msg = Twist()
    #     vel_msg.linear.x = 0
    #     vel_msg.linear.y = 0
    #     self.vel_pub.publish(vel_msg)

if __name__ == "__main__":
    node = SwarmInterface()
    node.run()