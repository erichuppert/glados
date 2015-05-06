#!/usr/bin/python2
import sys
import pygame
from threading import Lock,Thread
from pygame.locals import *
from math import *
from subprocess import Popen,PIPE
import re

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import message_filters

# Robot Dimensions
M_PER_INCH = 0.0254
ROBOT_WIDTH_INCHES = 15
ROBOT_HEIGHT_INCHES = 5
ROBOT_WIDTH = ROBOT_WIDTH_INCHES*M_PER_INCH
ROBOT_HEIGHT = ROBOT_HEIGHT_INCHES*M_PER_INCH

x_min = 0.0
x_max = 4.6419
y_min = 0.0
y_max = 3.048
width,height = 1500,1500
coords_to_screen = lambda x,y: (
    int((x-x_min)*width/(x_max-x_min)),
    int((y_max-y)*height/(y_max-y_min))
)
screen_to_coords = lambda x,y: (
    x*(x_max-x_min)/width + x_min,
    y_max - (y*(y_max-y_min)/height)
)
FPS = 10

surface_lock = Lock()
drawables = []
white = pygame.Color(255,255,255)
blue  = pygame.Color(0,0,255)
green = pygame.Color(0,255,0)
red   = pygame.Color(255,0,0)
nameToColor = {
    'white' : white,
    'blue'  : blue,
    'green' : green,
    'red'   : red
}
done = False
simulator = None

def get_input_objects():
    global drawables,done,simulator
    number = "(-?\\d+(?:\\.\\d+)?(?:e[+-]\\d+)?)"
    point_pattern = re.compile(
        "^Point ([a-z]+) %s %s$" % (number,number)
    )
    segment_pattern = re.compile(
        "^Segment ([a-z]+) %s %s %s %s$" % (number, number, number, number)
    )
    robot_pattern = re.compile(
        "^Robot ([a-z]+) %s %s %s$" % (number, number, number)
    )
    reset_pattern = re.compile(
        "^RESET (-?\\d+)$"
    )
    quit_pattern = re.compile(
        "^QUIT$"
    )
    while not done:
        inp = simulator.stdout.readline()
        #inp = raw_input()
        p = point_pattern.match(inp)
        s = segment_pattern.match(inp)
        r = robot_pattern.match(inp)
        rst = reset_pattern.match(inp)
        qut = quit_pattern.match(inp)
        if p:
            color = nameToColor[p.group(1)]
            x,y = coords_to_screen(float(p.group(2)), float(p.group(3)))
            with surface_lock:
                drawables.append((lambda x,y,color:
                                  lambda s: pygame.draw.circle(s,color,(x,y),10))
                                 (x,y,color))
        elif s:
            color = nameToColor[s.group(1)]
            x1,y1 = coords_to_screen(float(s.group(2)), float(s.group(3)))
            x2,y2 = coords_to_screen(float(s.group(4)), float(s.group(5)))
            with surface_lock:
                drawables.append((lambda x1,y1,x2,y2,color:
                                  lambda s: pygame.draw.line(s,color,(x1,y1),(x2,y2)))
                                 (x1,y1,x2,y2,color))
        elif r:
            color = nameToColor[r.group(1)]
            rx,ry,theta = float(r.group(2)),float(r.group(3)),float(r.group(4))
            rect = [
                ( ROBOT_HEIGHT/2.0 ,  ROBOT_WIDTH/2.0),
                ( ROBOT_HEIGHT/2.0 , -ROBOT_WIDTH/2.0),
                (-ROBOT_HEIGHT/2.0 , -ROBOT_WIDTH/2.0),
                (-ROBOT_HEIGHT/2.0 ,  ROBOT_WIDTH/2.0),
                ( ROBOT_HEIGHT/2.0 ,  ROBOT_WIDTH/2.0),
                ( ROBOT_HEIGHT/2.0 , 0               ),
                ( 0                , 0               )
            ]
            rect = [
                coords_to_screen(rx+x*cos(theta)-y*sin(theta),ry+x*sin(theta)+y*cos(theta))
                for (x,y) in rect
            ]
            with surface_lock:
                drawables.append((lambda rect,color:
                                  lambda s: pygame.draw.lines(s,color,False,rect))
                                 (rect[:],color))
        elif rst:
            with surface_lock:
                drawables.pop(int(rst.group(1)))
        elif qut:
            done = True
        else:
            print inp,"INVALID SYNTAX"

def update(scan,odom):
    global simulator
    pose = odom.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,r_theta = euler_from_quaternion(quat)

    (rx,ry) = (pose.position.x,pose.position.y)

    simulator.stdin.write("%f %f %f\n" % (rx+0.6,ry+0.6,r_theta))
    for r in scan.ranges:
        simulator.stdin.write("%f ")
    simulator.stdin.write("\n")

def spin_thread():
    rospy.spin()

def main(args):
    global drawables,done,simulator
    pygame.init()
    fpsClock = pygame.time.Clock()

    surface = pygame.display.set_mode((width,height))
    pygame.display.set_caption("Robot")

    simulator = Popen(["./simulator"], stdin=PIPE,stdout=PIPE)
    drawables = []
    comm_thread = Thread(target=get_input_objects)
    comm_thread.start()

    rospy.init_node("localization")
    scan_sub = message_filters.Subscriber("pcl_scan", LaserScan, queue_size=1)
    odom_sub = message_filters.Subscriber("odom", Odometry, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([scan_sub,odom_sub], 1, 0.1)
    ts.registerCallback(update)
    Thread(target=spin_thread).start()

    while not done:
        surface.fill(white)
        with surface_lock:
            list(d(surface) for d in drawables)

        for event in pygame.event.get():
            if event.type == QUIT:
                done = True

        fpsClock.tick(FPS)
        pygame.display.update()

    pygame.quit()
    simulator.terminate()
    simulator.wait()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
