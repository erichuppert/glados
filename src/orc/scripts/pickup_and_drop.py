#!/usr/bin/env python2
import rospy
import math
from orc.msg import JointSet


state= "start"
joint = JointSet()

def handleAngleMessage(jointState):
    global state
    name = jointState.name
    angle = jointState.angle
    angleStates[name] = angle
    if state == "start" or checkAngles():
    	# move to next state
         state = nextState(state)
        
angleStates = {
    "base_to_shoulder" : None, 
    "shoulder_to_wrist": None,
    "wrist_to_gripper" : None
}   

desiredStates = {
    "base_to_shoulder" : None, 
    "shoulder_to_wrist": None,
    "wrist_to_gripper" : None
} 

def checkAngles():
    angleChecks = [abs(desiredStates[x] - angleStates[x]) < 0.1 for x in angleStates]
    return all(angleChecks)

def set_angle(name,angle):
    desiredStates[name] = angle
    joint.joint_name=name
    joint.angle=angle
    joint_pub_.publish(joint)
    
states = [ "findblock","grip","pickup","drop","stop"] 
stateAngles = {
    "findblock" : (-1.3,0.8,0.8),"grip":(2.0,1.24,0.55),"pickup":(2.0,1.24,0.55),"drop":(2.0,1.24,0.8)
    }

def nextState (state):
    currentIndex = states.index(state)
    if currentIndex == len(states)-1:
    	raise "Foo"
    newState = states[currentIndex + 1]
    set_angle("base_to_shoulder", stateAngles[newState][0])
    set_angle("shoulder_to_wrist",stateAngles[newState][1])
    set_angle("wrist_to_gripper",stateAngles[newState][2])
    return newState

   
def main():
	global joint_pub_
	rospy.init_node('Pickup_and_drop')
	joint_pub_=rospy.Publisher("/joint_set",100)
        rospy.Subscriber("joint_state",JointState, handleAngleMessage)
if __name__ == "__main__":
    main()
