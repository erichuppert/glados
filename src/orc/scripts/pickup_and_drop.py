#!/usr/bin/env python2
import rospy
import math
from orc.msg import JointSet
from sensor_msgs.msg import JointState
from orc.srv import PickupBlock, PickupBlockResponse

state= "waiting"
joint = JointSet()

def pickupAndDrop(req):
    global state
    loop = rospy.Rate(30)
    state = "start"
    while state != "stop":
        loop.sleep()
    state = "waiting"
    #calling door service
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(2)
    except rospy.ServiceException, e:
        print "Service call failed"
    return PickupBlockResponse()

def handleAngleMessage(jointState):
    global state,angleStates
    angleStates = dict(zip(jointState.name,jointState.position))
    if (state == "stop" or state == "waiting"):
        return
    if state == "start" or checkAngles():
    	# move to next state
        state = nextState(state)
    if (state == "stop"):
        return
    for (name, angle) in zip(joint_names,stateAngles[state]):
        set_angle(name,angle)

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
    angleChecks = [angleStates[x] is not None and desiredStates[x] is not None and abs(desiredStates[x] - angleStates[x]) < 0.01 for x in desiredStates]
    return all(angleChecks)

def set_angle(name,angle):
    global desiredStates
    desiredStates[name] = angle
    joint.joint_name=name
    joint.angle=angle
    joint_pub_.publish(joint)

states = ["waiting", "start", "findblock","grip","pickup","drop","stop"]
joint_names = ["base_to_shoulder", "shoulder_to_wrist", "wrist_to_gripper"]
stateAngles = {
    "findblock" : (-1.3,0.80,0.8),
    "grip"      : (-1.3,0.80,0.4),
    "pickup"    : ( 2.0,1.24,0.4),
    "drop"      : ( 2.0,1.24,0.8)
}

def nextState (state):
    currentIndex = states.index(state)
    if currentIndex == len(states)-1:
    	raise "Foo"
    return states[currentIndex + 1]

def main():
	global joint_pub_
	rospy.init_node('Pickup_and_drop_server')
        s = rospy.Service('PickupAndDrop', PickupBlock, pickupAndDrop)
	joint_pub_=rospy.Publisher("joint_set",JointSet, queue_size=100)
        rospy.Subscriber("joint_states",JointState, handleAngleMessage)
        rospy.spin()
if __name__ == "__main__":
    main()
