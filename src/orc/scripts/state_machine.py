#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Pose
from orc.srv import OpenCloseDoorResponse, OpenCloseDoor, OpenCloseDoorRequest, VisualServo, VisualServoRequest, Collides, CollidesResponse
import tf
import math

blocks_held = 0

DOOR_OPEN = 1
DOOR_CLOSED = 0

EXPLORER_ON = 1
EXPLORER_OFF = 0

def change_explorer_state(val):
    msg = UInt16()
    msg.data = val
    # explorer_state_pub.publish(msg)

def pickup_block():
    # this should synchronously call visual servoing to start, which should make the arm
    # pickup the block when the bump sensors are triggered
    rospy.wait_for_service('VisualServo')
    try:
        visual_servo_service = rospy.ServiceProxy('VisualServo',VisualServo)
        visual_servo_service()
    except rospy.ServiceException, e:
        print "Visual Servo service call failed"
    return True
    
def change_door_state(state):
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(state) # not sure about this.
    except rospy.ServiceException, e:
        print "Door service call failed"

def get_destination_pose_msg(block_point_msg):
    x, y = block_point_msg.point.x, block_point_msg.point.y
    angle_to_block = math.atan2(y,x)
    pose_msg = Pose()
    pose_msg.position = block_point_msg.point
    pose_msg.orientation = Quaternion()
    pose_msg.orientation.x = x
    pose_msg.orientation.y = y
    pose_stamped = PoseStamped()
    pose_stamped.header = block_point_msg.header
    pose_stamped.pose = pose_msg
    return pose_stamped

def will_colide_with(point_msg):
    collision_service = rospy.ServiceProxy("collides", Collides)
    destination_pose_base_frame = get_destination_pose_msg(point_msg)
    odometry_pose = listener.transformPose("odom", destination_pose_base_frame)
    destination_angle = math.atan2(destination_pose_base_frame.pose.orientation.y, destination_pose_base_frame.pose.orientation.x)
    collision_response = collision_service(odometry_pose.pose.position.x, odometry_pose.pose.position.y, destination_angle)
    print "Collision detection result: " +  str(collision_response.collides)
    return collision_response.collides

def handle_nearest_block_msg(point_msg):
    global blocks_held
    print "Block seen. Checking if reachable"
    if not will_colide_with(point_msg):
        print "block is reachable, stopping explorer"
        change_explorer_state(EXPLORER_OFF)
        # after we release the tower, we don't close the door, so make sure its closed before we put a block inside
        print "trying to pickup block"
        if pickup_block():
            blocks_held += 1
            print "Now holding %d blocks" % blocks_held
            if blocks_held == 6:
                print "releasing tower now"
                change_door_state(DOOR_OPEN)
                blocks_held = 0
                print "tower released"
        print "resuming exploration"
        change_explorer_state(EXPLORER_ON)
    else:
        print "Skipping unreachable block"

def main():
    global nearest_block_sub, explorer_state_pub, listener
    rospy.init_node('state_machine')
    explorer_state_pub = rospy.Publisher("explorer_state", UInt16, queue_size=10)
    nearest_block_sub = rospy.Subscriber("nearest_block", PointStamped, handle_nearest_block_msg, queue_size=1)
    listener = tf.TransformListener();
    change_explorer_state(EXPLORER_ON)
    rospy.spin()
    
    
if __name__ == "__main__":
    main()
