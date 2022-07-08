#!/usr/bin/env python3
from pickle import TRUE
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import tf
import numpy as np
from models.Boids import Boids as bo
from utils.swarm_utils import neighborHood
import copy

# eventually could be rosparams
class Params:
    def __init__(self,active_robots = [],max_vel=0.01,max_accel=np.inf,max_turn_rate=np.inf,edges = 0.1,neighbor_radius=0.05,controller=bo(1,1,1)):
        self.active_robots = active_robots
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_turn_rate = max_turn_rate
        self.edges = edges # top bottom left right
        self.neighbor_radius = neighbor_radius
        self.controller = controller

#is it controlled now, let's go, hm
class SwarmInterface(object):
    def __init__(self,params=Params()):
        rospy.init_node('swarm_interface')
        self.params = params
        self.listener = tf.TransformListener()
        self.agent_vel_pubs = []
        self.agent_positions = np.zeros((len(params.active_robots),2))#might need better init for vels
        self.agent_vels = np.zeros((len(params.active_robots),2))#might need better init

        self.controllers = [copy.deepcopy(params.controller) for i in range(len(params.active_robots))] #eventually parameterize input
        for i in range(len(self.params.active_robots)):
            id = str(self.params.active_robots[i])
            rospy.loginfo('Publisher:/turtle'+id+'/cmd_vel')
            pub = rospy.Publisher('/turtle'+id+'/cmd_vel', Twist, queue_size=10)
            # pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

            self.agent_vel_pubs.append(pub)
            # initial velocities
            msg = Twist()
            vec = np.random.uniform(size=2)
            vec = vec/np.linalg.norm(vec) * params.max_vel#np.random.uniform(low=0,high=self.params.max_vel) # start them all at 0-max vel in a random direction
            rospy.loginfo("Inited to "+str(vec[0])+" "+str(vec[1]))

            
            msg.linear.x = vec[0]
            msg.linear.y = vec[1]

            # wait for publisher to connect before sending inits
            while pub.get_num_connections() == 0:
                rospy.loginfo("Waiting for subscriber to connect")
                rospy.sleep(1)
            pub.publish(msg)
            self.agent_vels[i]=vec #just use last commanded vels instead of real
        
    
    def run(self):
        rateHz = 100 # might need to speed up
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                # get all the agent positions
                for i in range(len(self.params.active_robots)):
                    id = str(self.params.active_robots[i])
                    (trans,rot) =  self.listener.lookupTransform('/world','/vicon/turtle'+id+'/turtle'+id, rospy.Time(0))
                    currentPos = np.array([trans[0],trans[1]])
                    dt = 1/rateHz
                    # self.agent_vels[i] = (currentPos - self.agent_positions[i])/dt #figure out a dt calculation with the rate--this might become problematic
                    self.agent_positions[i] = currentPos

            except:
                continue
            
            # now velocities based on controller
            for i in range(len(self.params.active_robots)):
                agentPos = self.agent_positions[i]
                agentVel = self.agent_vels[i]
                # import neighborhood function
                relevantPositions, relevantVelocities = [],[]
                
                if len(params.active_robots)>1:
                    relevantPositions, relevantVelocities =  neighborHood(agentPos,self.params.neighbor_radius,self.agent_positions,self.agent_vels)

                # default to last one
                next_vel = agentVel
                # BCs , touch grass
                id = str(self.params.active_robots[i])
                rospy.loginfo("Agent "+id)

                outside = True

                if self.params.edges[3]<agentPos[0]:
                    rospy.loginfo("Out of Bounds X Positive(Right)")
                    if next_vel[0]>0:next_vel[0]*=-1 
                elif self.params.edges[2]>agentPos[0]:
                    rospy.loginfo("Out of Bounds X Negative(Left)")
                    if next_vel[0]<0:next_vel[0]*=-1 
                elif self.params.edges[0]<agentPos[1]:
                    rospy.loginfo("Out of Bounds Y Positive(Top)")
                    if next_vel[1]>0:next_vel[1]*=-1 
                elif self.params.edges[1]>agentPos[1]:
                    rospy.loginfo("Out of Bounds Y Negative(Bottom")
                    if next_vel[1]<0:next_vel[1]*=-1 
                else:
                    rospy.loginfo("In Bounds")
                    outside=False

                rospy.loginfo("Length of relevant poses:"+str(len(relevantPositions)))
                if not outside:
                    next_vel = self.controllers[i].vel(relevantPositions,relevantVelocities,agentPos,agentVel)
                # eventually use dedicated method
                if np.linalg.norm(next_vel) > self.params.max_vel:
                    next_vel *= (self.params.max_vel/np.linalg.norm(next_vel))

                #publish
                vel_msg = Twist()
                vel_msg.linear.x = next_vel[0]
                vel_msg.linear.y = next_vel[1]
                self.agent_vel_pubs[i].publish(vel_msg)
                self.agent_vels[i] = next_vel
            # if self.params.edges<trans[0]:
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
    params = Params(controller=bo(3,2,0.1),active_robots=[1,2,4,5,6],neighbor_radius=0.4,edges=[0.2,-0.2,-0.8,0.3],max_vel=0.15)
    node = SwarmInterface(params=params)
    node.run()