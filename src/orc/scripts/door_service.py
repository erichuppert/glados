import rospy, math
from orc.srv import OpenCloseDoor
from sensor_msgs.msg import JointState

OPEN_ANGLE = math.pi/2.0
CLOSED_ANGLE = 0.0

EPSILON = 0.1

door_angle = 0.0

def open_close_door(req):
    if req.state == 1:
        desired_angle = OPEN_ANGLE
    else:
        desired_angle = CLOSED_ANGLE
    while abs(door_angle - desired_angle) > EPSILON:
        pass
    return
            

def handle_joint_msg(msg):
    door_servo_index = msg.name.index("door")
    door_angle = msg.position[door_servo_index]

def main():
    rospy.init_node('door_open_close_server')
    s = rospy.Service('open_close_door', AddTwoInts, open_close_door)
    joint_state_pub = rospy.Publisher("joint_set", JointStateMsg, queue_size=10)
    joint_angle_sub = rospy.Subscriber("joint_state", JointMsg, hande_joint_msg)
    rospy.spin()

if __name__ == '__main__':
    main()
        
