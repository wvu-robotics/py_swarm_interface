import numpy as np

#note agentPositions and Vels at a single time
def neighborHood(pos,radius,agentPositions,agentVelocities):
    # fanciness might be unnessecary and slower, but it's short
    rel_pos = agentPositions - pos
    dist = np.linalg.norm(rel_pos,axis=1)
    inRadius = np.bitwise_and(dist <= radius, dist>0) #boolean array
   
    inRadius = np.repeat(inRadius,2) #make dimensions match with agentPositions and vels
    return np.extract(inRadius,agentPositions).reshape(-1,2),np.extract(inRadius,agentVelocities).reshape(-1,2)


# def motionConstraints(vel_new,vel_last,params=SimParams()):
#     maxAngleDeviation = params.agent_max_turn_rate*params.dt
#     maxVelChange = params.agent_max_accel*params.dt

#     #angle deviation of velocity
#     if(np.linalg.norm(vel_new) > 0 and np.linalg.norm(vel_last) > 0):
#         vel_new_angle = np.arctan2(vel_new[1],vel_new[0])
#         vel_angle = np.arctan2(vel_last[1],vel_last[0])
#         angleDeviation = vel_new_angle - vel_angle
#         if angleDeviation > np.pi or angleDeviation < -np.pi:
#             angleDeviation = -2*np.pi + angleDeviation

#         if abs(angleDeviation) > maxAngleDeviation:
#             agent_vel_hat = vel_last/np.linalg.norm(vel_last)
#             agent_vel_complex = agent_vel_hat[0] + 1j*agent_vel_hat[1]
#             agent_vel_complex *= np.linalg.norm(vel_new)
#             # figure out which side to rotate
#             if vel_angle + maxAngleDeviation > np.pi:
#                 maxAngleDeviation += 2*np.pi
#             elif vel_angle - maxAngleDeviation < -1*np.pi:
#                 maxAngleDeviation -= 2*np.pi

#             if angleDeviation > 0:
#                 agent_vel_complex *= cmath.exp(1j*-1*maxAngleDeviation)
#             elif angleDeviation < 0:
#                 agent_vel_complex *= cmath.exp(1j*maxAngleDeviation)
#             else:
#                 pass
#             vel_new = np.array([agent_vel_complex.real,agent_vel_complex.imag])
    # #max acceleration
    # #have to account for going down to zero
    # if np.linalg.norm(vel_new)-np.linalg.norm(vel_last) > maxVelChange:
    #     if np.linalg.norm(vel_new) == 0:
    #         vel_new = vel_last + maxVelChange*(vel_last/np.linalg.norm(vel_last))
    #     else:
    #         vel_new = vel_new*((maxVelChange+np.linalg.norm(vel_last))/np.linalg.norm(vel_new)) 
    # elif np.linalg.norm(vel_new)-np.linalg.norm(vel_last) < -maxVelChange:
    #     if np.linalg.norm(vel_new) == 0:
    #         vel_new = vel_last - maxVelChange*(vel_last/np.linalg.norm(vel_last))
    #     else:
    #         vel_new = vel_new*((-maxVelChange+np.linalg.norm(vel_last))/np.linalg.norm(vel_new))

    # #max vel
    # if np.linalg.norm(vel_new) > params.agent_max_vel:
    #     vel_new *= (params.agent_max_vel/np.linalg.norm(vel_new))

    # return vel_new