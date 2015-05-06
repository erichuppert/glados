#include "ros/ros.h"
#include "ransac_lines.h"
#define RRT_ITER_COUNT 1000

vector<Segment> obstacle_segments;

bool rrt_and_search(orc::RRT::Request &req, orc::RRT::Response &res) {
  geometry_msgs::Pose current_pose;
  geometry_msgs::Pose destination_pose = req.destination;
  std::vector<Node> all_nodes;
  Node start_node(current_pose);
  Node nearest_neighbor;
  all_nodes.push_back(start_node);
  for (int i=0; i<RRT_ITER_COUNT; i++) {
    float rand_x = 0;
    float rand_y = 0;
    float rand_theta = 0;
    for (auto const &node : all_nodes) {
      if squared_distance(node, random_node) < squared_distance(nearest_neighbor, random_node) {
	  nearest_neighbor = node;
	}
    }
    if (!path_collides(random_node, nearest_neighbor)) {
      nearest_neighbor.neighbors.insert(random_node);
      all_nodes.add(random_node);
    }
  }

  req.path = a_star(start_node, destination_pose);
  return true;
}

bool path_collides(Node &node1, Node &node2) {
  geometry_msgs::Point p1 = n1.position;
  geometry_msgs::Point p2 = n2.position;
  Segment line_between(Point(p1.x,p1.y), Point(p2.x,p2.y));
  for (auto const &segment : segments) {
    if (line_between.intersects(segment)) {
      return true;
    }
  }
  return false;
}

float squared_distance(Node n1, Node n2) {
  geometry_msgs::Point p1 = n1.position;
  geometry_msgs::Point p2 = n2.position;
  float squared_distance = 0.0;
  squared_distance += pow((p1.x - p2.x),2);
  squared_distance += pow((p1.y - p2.y),2);
  return squared_distance;
}

vector<geometry_msgs::pose> a_star(Node &start_node, geometry_msgs::Pose goal_pose) {
  vector<geometry_msgs::pose> path;
  
  return path;
}

std::vector<Segment> get_map() {
    std::vector<Segment> segments;
    segments.push_back(Segment(Point(0.000000, 0.000000),Point(0.000000, 3.048000)));
    segments.push_back(Segment(Point(0.000000, 3.048000),Point(4.641900, 3.048000)));
    segments.push_back(Segment(Point(4.641900, 3.048000),Point(4.641900, 0.000000)));
    segments.push_back(Segment(Point(4.641900, 0.000000),Point(0.000000, 0.000000)));
    segments.push_back(Segment(Point(0.000000, 1.460000),Point(0.000000, 1.670000)));
    segments.push_back(Segment(Point(0.000000, 1.670000),Point(1.470000, 1.670000)));
    segments.push_back(Segment(Point(1.470000, 1.670000),Point(1.470000, 1.060000)));
    segments.push_back(Segment(Point(1.470000, 1.060000),Point(1.260000, 1.060000)));
    segments.push_back(Segment(Point(1.260000, 1.060000),Point(1.260000, 1.470000)));
    segments.push_back(Segment(Point(1.260000, 1.470000),Point(0.000000, 1.460000)));
    segments.push_back(Segment(Point(3.730000, 2.390000),Point(4.060000, 1.860000)));
    segments.push_back(Segment(Point(4.060000, 1.860000),Point(4.130000, 1.790000)));
    segments.push_back(Segment(Point(4.130000, 1.790000),Point(3.720000, 1.340000)));
    segments.push_back(Segment(Point(3.720000, 1.340000),Point(3.030000, 1.940000)));
    segments.push_back(Segment(Point(3.030000, 1.940000),Point(3.090000, 2.030000)));
    segments.push_back(Segment(Point(3.090000, 2.030000),Point(3.730000, 2.390000)));
    segments.push_back(Segment(Point(0.295000, 0.000000),Point(0.295000, 0.110000)));
    segments.push_back(Segment(Point(0.295000, 0.110000),Point(1.520000, 0.110000)));
    segments.push_back(Segment(Point(1.520000, 0.110000),Point(1.520000, 0.000000)));
    segments.push_back(Segment(Point(1.520000, 0.000000),Point(0.295000, 0.000000)));
    segments.push_back(Segment(Point(2.060000, 0.000000),Point(3.160000, 0.000000)));
    segments.push_back(Segment(Point(3.160000, 0.000000),Point(2.510000, 0.550000)));
    segments.push_back(Segment(Point(2.510000, 0.550000),Point(2.060000, 0.000000)));
    segments.push_back(Segment(Point(2.060000, 0.000000),Point(3.160000, 0.000000)));
    segments.push_back(Segment(Point(3.160000, 0.000000),Point(2.510000, 0.550000)));
    segments.push_back(Segment(Point(2.510000, 0.550000),Point(2.060000, 0.000000)));
    segments.push_back(Segment(Point(0.000000, 2.970000),Point(0.620000, 2.440000)));
    segments.push_back(Segment(Point(0.620000, 2.440000),Point(0.690000, 2.520000)));
    segments.push_back(Segment(Point(0.690000, 2.520000),Point(0.100000, 3.050000)));
    segments.push_back(Segment(Point(0.100000, 3.050000),Point(0.000000, 2.970000)));
    segments.push_back(Segment(Point(2.000000, 3.050000),Point(1.480000, 2.330000)));
    segments.push_back(Segment(Point(1.480000, 2.330000),Point(1.390000, 2.370000)));
    segments.push_back(Segment(Point(1.390000, 2.370000),Point(1.900000, 3.050000)));
    segments.push_back(Segment(Point(1.900000, 3.050000),Point(2.000000, 3.050000)));
    segments.push_back(Segment(Point(2.240000, 1.830000),Point(2.340000, 1.830000)));
    segments.push_back(Segment(Point(2.340000, 1.830000),Point(2.340000, 3.050000)));
    segments.push_back(Segment(Point(2.340000, 3.050000),Point(2.240000, 3.050000)));
    segments.push_back(Segment(Point(2.240000, 3.050000),Point(2.240000, 1.830000)));
    return segments;
}

int main() {
  obstacle_segments = get_map();
  
  ros::init(argc, argv, "rrt_service");
  ros::NodeHandle nh;

  ros::ServiceServer = nh.advertiseService("rrt_and_search", rrt_and_search);
  ros::spin();

  return 0;
}

Class Node {
  std::set<Node> neighbors;
  geometry_msgs::Pose pose;
  float priority = 0.0;
  
  Node::Node(geometry_msgs::Pose &_pose) {
    pose = _pose;
  }

  bool operator<(const node & a, const node & b) {
    return a.priority > b.priority;
  }
}
