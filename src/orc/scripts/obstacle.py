#!/usr/bin/python2
import rospy, math
from orc.srv import ObstacleSegments_service, ObstacleSegmentResponse
from orc.msg import ObstacleSegments
from sensor_msgs.msg import 
from sympy import Point,Line


#obstacle 1
p0,p1,p2,p3,p4,p5 = Point(0.0,1.46),Point(0.0,1.67),Point(1.47,1.67),Point(1.47,1.06),Point(1.26,1.06),Point(1.26,1.47)
l0,l1,l2,l3,l4,l5 = Line(p0,p1),Line(p1,p2),Line(p2,p3),Line(p3,p4),Line(p4,p5),Line(p5,p0)

#obstacle 2 
p6,p7,p8,p9,p10 = Point(0.295,00),Point(0.295,0.11),Point(1.52,0.11),Point(1.52,0.0)
l6,l7,l8,l9 = Line(p6,p7),Line(p7,p8),Line(p8,p9),Line(p9,p6)

#obstacle 3
p11,p12,p13 = Point(2,06,0.0),Point(3.16,0.0),Point(2.51,0.55)
l10,l11,l12 = Line(p11,p12),Line(p12,p13),Line(p13,p11)

#obstacle 4
p14,p15,p16,p17 = Point(0.0,2.97),Point(0.62,2.44),Point(0.69,2.52),Point(0.1,3.05)
l13,l14,l15,l16 = Line(p14,p15),Line(p15,p16),Line(p16,p17),Line(p17,p14)

#obstacle 5
p18,p19,p20,p21 = Point(2.0,3.05),Point(1.48,2.33),Point(1.39,2.37),Point(1.90,3.05)
l17,l18,l19,l20 = Line(p18,p19),Line(p19,p20),Line(p20,p21),Line(p21,p18)

#obstacle 6
p22,p23,p24,p25 = Point(2.24,1.83),Point(2.34,1.83),Point(2.34,3.05),Point(2.24,3.05)
l21,l22,l23,l24 = Line(p22,p23),Line(p23,p24),Line(p24,p25),Line(p25,p22)

#obstacle 7
p26,p27,p28,p29,p30,p31 = Point(3.73,2.39),Point(4.06,1.86),Point(4.13,1.79),Point(3.72,1.34),Point(3.03,1.94),Point(3.09 2.03)
l25,l26,l27,l28,l29,l30 = Line(p26,p27),Line(p27,p28),Line(p28,p29),Line(p29,p30),Line(p30,P31),Line(p31,p26)

Segments = [l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11,l12,l13,l14,l15,l16,l17,l18,l19,l20,l21,l22,l23,l24,l25,l26,l27,l28,l29,l30]

def ObstacleSegments (req):
	global Segments
	obstacles = ObstacleSegments()
	obstacles.segments = Segments
	obstacle_pub.publish (obstacles)

def main():
	rospy.init_node('ObstacleSegments_server')
	 s = rospy.Service('ObstacleSegments', ObstacleSegments, obstacle_segments)
	obstacle_pub=rospy.Publisher("obstacle_segments",obstacle_segments)
if __name__ == '__main__':
	main()

