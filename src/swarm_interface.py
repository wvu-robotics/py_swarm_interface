#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import tf
import numpy as np

# eventually could be rosparams
class Params:
    def __init__(self,activeRobots = [],max_vel=0.01,max_accel=np.inf,max_turn_rate=np.inf,edge = 0.1,neighbor_radius=0.05):
        self.activeRobots = activeRobots
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_turn_rate = max_turn_rate
        self.edge = edge
        self.neighbor_radius = neighbor_radius

#is it controlled now, let's go, hm
class SwarmInterface(object):
    def __init__(self,params=Params()):
        rospy.init_node('swarm_interface')
        self.params = params
        self.listener = tf.TransformListener()
        self.test_pub = rospy.Publisher('/test_topic', String, queue_size=10)
        self.agent_vel_pubs = []
        self.agent_positions = np.zeros((len(params.activeRobots),2))#might need better init for vels
        self.agent_vels = np.zeros((len(params.activeRobots),2))#might need better init

        self.controllers = [] #eventually parameterize input
        for i in self.params.active_robots:
            self.agent_vel_pubs.append(rospy.Publisher('/turtle'+str(i)+'/cmd_vel', Twist, queue_size=10))
            # initial velocities
            msg = Twist()
            vec = np.uniform.random(size=2)
            vec = vec/np.linalg.norm(vec) * np.uniform.random(low=0,high=self.params.max_vel) # start them all at 0-max vel in a random direction
            msg.linear.x = vec[0]
            msg.linear.y = vec[1]
            self.agent_vel_pubs[i].publish(msg)
        
        self.edges = .20 #change to rectangular later
        # self.wasLastInBounds = True
    
    def run(self):
        rateHz = 10 # might need to speed up
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                # get all the agent positions
                for i in range(len(self.params.activeRobots)):
                    id = str(self.params.activeRobots[i])
                    (trans,rot) =  self.listener.lookupTransform('/world','/vicon/turtle'+id+'/turtle'+id, rospy.Time(0))
                    currentPos = np.array([trans[0],trans[1]])
                    dt = 1/rateHz
                    self.agent_vels[i] = (currentPos - self.agent_positions[i])/dt #figure out a dt calculation with the rate--this might become problematic
                    self.agent_positions[i] = currentPos

            except:
                continue
            
            # now velocities based on controller
            for i in range(len(self.params.activeRobots)):
                agentPos = self.agent_positions[i]
                agentVel = self.agent_vels[i]
                # import neighborhood function
                relevantPositions, relevantVelocities = getNeighborhood(self.agent_positions,self.agent_vels,self.params.neighbor_radius,agentPos)

                # default to last one
                next_vel = agentVel
                # BCs , touch grass
                outside = False
                if self.edges<agentVel[0]:
                    rospy.loginfo("Out of Bounds X Positive")
                    if next_vel[0]>0:next_vel[0]*=-1 
                if -self.edges>agentVel[0]:
                    rospy.loginfo("Out of Bounds X Negative")
                    if next_vel[0]<0:next_vel[0]*=-1 
                if self.edges<agentVel[1]:
                    rospy.loginfo("Out of Bounds Y Positive")
                    if next_vel[1]>0:next_vel[1]*=-1 
                if -self.edges>agentVel[1]:
                    rospy.loginfo("Out of Bounds Y Negative")
                    if next_vel[1]<0:next_vel[1]*=-1 
                else:
                    rospy.loginfo("In Bounds")

                if not outside:
                    next_vel = self.controllers[i].get_vel(relevantPositions,relevantVelocities,agentPos,agentVel)

                

                #publish
                vel_msg = Twist()
                vel_msg.x = next_vel[0]
                vel_msg.y = next_vel[1]
                # self.agent_vel_pubs[i].publish(vel_msg)

            # if self.edges<trans[0]:
            #     rospy.loginfo("Out of Bounds X Positive")
            #     if self.vel[0]>0:self.vel[0]*=-1 
            # if -self.edges>trans[0]:
            #     rospy.loginfo("Out of Bounds X Negative")
            #     if self.vel[0]<0:self.vel[0]*=-1 
            # if self.edges<trans[1]:
            #     rospy.loginfo("Out of Bounds Y Positive")
            #     if self.vel[1]>0:self.vel[1]*=-1 
            # if -self.edges>trans[1]:
            #     rospy.loginfo("Out of Bounds Y Negative")
            #     if self.vel[1]<0:self.vel[1]*=-1 
            # else:
            #     self.wasLastInBounds = True
            #     rospy.loginfo("In Bounds")
            # rospy.loginfo("Current vel" + str(self.vel[0]) +" "+ str(self.vel[1]))

            # vel_msg = Twist()
            # vel_msg.linear.x = self.vel[0]
            # vel_msg.linear.y = self.vel[1]
            # self.vel_pub.publish(vel_msg)
            # self.test_pub.publish("test "+str(trans[0])+" "+str(trans[1])+" "+str(trans[2]))
            rate.sleep()

if __name__ == "__main__":
    params = Params(activeRobots=[2,4],neighbor_radius=0.05,edges=0.2,max_vel=0.01)
    node = SwarmInterface()
    node.run()