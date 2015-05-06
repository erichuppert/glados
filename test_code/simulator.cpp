using namespace std;

#include "ransac_lines.h"
#include "particle_filter.h"
#include <vector>
#include <iostream>
#include <unistd.h>
#include <random>
#include <cstdlib>

vector<Segment> field;

// Robot Dimensions
//
#define M_PER_INCH          0.0254
#define ROBOT_WIDTH_INCHES  15
#define ROBOT_HEIGHT_INCHES 10
#define ROBOT_WIDTH         ROBOT_WIDTH_INCHES*M_PER_INCH
#define ROBOT_HEIGHT        ROBOT_HEIGHT_INCHES*M_PER_INCH
#define DEV_PER_M           0.001

void build_map() {
    field.push_back({{0.000000,0.000000},{0.000000,3.048000}});
    cout << "Segment blue 0.000000 0.000000 0.000000 3.048000" << endl;
    field.push_back({{0.000000,3.048000},{4.641900,3.048000}});
    cout << "Segment blue 0.000000 3.048000 4.641900 3.048000" << endl;
    field.push_back({{4.641900,3.048000},{4.641900,0.000000}});
    cout << "Segment blue 4.641900 3.048000 4.641900 0.000000" << endl;
    field.push_back({{4.641900,0.000000},{0.000000,0.000000}});
    cout << "Segment blue 4.641900 0.000000 0.000000 0.000000" << endl;
    field.push_back({{0.000000,1.460000},{0.000000,1.670000}});
    cout << "Segment blue 0.000000 1.460000 0.000000 1.670000" << endl;
    field.push_back({{0.000000,1.670000},{1.470000,1.670000}});
    cout << "Segment blue 0.000000 1.670000 1.470000 1.670000" << endl;
    field.push_back({{1.470000,1.670000},{1.470000,1.060000}});
    cout << "Segment blue 1.470000 1.670000 1.470000 1.060000" << endl;
    field.push_back({{1.470000,1.060000},{1.260000,1.060000}});
    cout << "Segment blue 1.470000 1.060000 1.260000 1.060000" << endl;
    field.push_back({{1.260000,1.060000},{1.260000,1.470000}});
    cout << "Segment blue 1.260000 1.060000 1.260000 1.470000" << endl;
    field.push_back({{1.260000,1.470000},{0.000000,1.460000}});
    cout << "Segment blue 1.260000 1.470000 0.000000 1.460000" << endl;
    field.push_back({{3.730000,2.390000},{4.060000,1.860000}});
    cout << "Segment blue 3.730000 2.390000 4.060000 1.860000" << endl;
    field.push_back({{4.060000,1.860000},{4.130000,1.790000}});
    cout << "Segment blue 4.060000 1.860000 4.130000 1.790000" << endl;
    field.push_back({{4.130000,1.790000},{3.720000,1.340000}});
    cout << "Segment blue 4.130000 1.790000 3.720000 1.340000" << endl;
    field.push_back({{3.720000,1.340000},{3.030000,1.940000}});
    cout << "Segment blue 3.720000 1.340000 3.030000 1.940000" << endl;
    field.push_back({{3.030000,1.940000},{3.090000,2.030000}});
    cout << "Segment blue 3.030000 1.940000 3.090000 2.030000" << endl;
    field.push_back({{3.090000,2.030000},{3.730000,2.390000}});
    cout << "Segment blue 3.090000 2.030000 3.730000 2.390000" << endl;
    field.push_back({{0.295000,0.000000},{0.295000,0.110000}});
    cout << "Segment blue 0.295000 0.000000 0.295000 0.110000" << endl;
    field.push_back({{0.295000,0.110000},{1.520000,0.110000}});
    cout << "Segment blue 0.295000 0.110000 1.520000 0.110000" << endl;
    field.push_back({{1.520000,0.110000},{1.520000,0.000000}});
    cout << "Segment blue 1.520000 0.110000 1.520000 0.000000" << endl;
    field.push_back({{1.520000,0.000000},{0.295000,0.000000}});
    cout << "Segment blue 1.520000 0.000000 0.295000 0.000000" << endl;
    field.push_back({{2.060000,0.000000},{3.160000,0.000000}});
    cout << "Segment blue 2.060000 0.000000 3.160000 0.000000" << endl;
    field.push_back({{3.160000,0.000000},{2.510000,0.550000}});
    cout << "Segment blue 3.160000 0.000000 2.510000 0.550000" << endl;
    field.push_back({{2.510000,0.550000},{2.060000,0.000000}});
    cout << "Segment blue 2.510000 0.550000 2.060000 0.000000" << endl;
    field.push_back({{2.060000,0.000000},{3.160000,0.000000}});
    cout << "Segment blue 2.060000 0.000000 3.160000 0.000000" << endl;
    field.push_back({{3.160000,0.000000},{2.510000,0.550000}});
    cout << "Segment blue 3.160000 0.000000 2.510000 0.550000" << endl;
    field.push_back({{2.510000,0.550000},{2.060000,0.000000}});
    cout << "Segment blue 2.510000 0.550000 2.060000 0.000000" << endl;
    field.push_back({{0.000000,2.970000},{0.620000,2.440000}});
    cout << "Segment blue 0.000000 2.970000 0.620000 2.440000" << endl;
    field.push_back({{0.620000,2.440000},{0.690000,2.520000}});
    cout << "Segment blue 0.620000 2.440000 0.690000 2.520000" << endl;
    field.push_back({{0.690000,2.520000},{0.100000,3.050000}});
    cout << "Segment blue 0.690000 2.520000 0.100000 3.050000" << endl;
    field.push_back({{0.100000,3.050000},{0.000000,2.970000}});
    cout << "Segment blue 0.100000 3.050000 0.000000 2.970000" << endl;
    field.push_back({{2.000000,3.050000},{1.480000,2.330000}});
    cout << "Segment blue 2.000000 3.050000 1.480000 2.330000" << endl;
    field.push_back({{1.480000,2.330000},{1.390000,2.370000}});
    cout << "Segment blue 1.480000 2.330000 1.390000 2.370000" << endl;
    field.push_back({{1.390000,2.370000},{1.900000,3.050000}});
    cout << "Segment blue 1.390000 2.370000 1.900000 3.050000" << endl;
    field.push_back({{1.900000,3.050000},{2.000000,3.050000}});
    cout << "Segment blue 1.900000 3.050000 2.000000 3.050000" << endl;
    field.push_back({{2.240000,1.830000},{2.340000,1.830000}});
    cout << "Segment blue 2.240000 1.830000 2.340000 1.830000" << endl;
    field.push_back({{2.340000,1.830000},{2.340000,3.050000}});
    cout << "Segment blue 2.340000 1.830000 2.340000 3.050000" << endl;
    field.push_back({{2.340000,3.050000},{2.240000,3.050000}});
    cout << "Segment blue 2.340000 3.050000 2.240000 3.050000" << endl;
    field.push_back({{2.240000,3.050000},{2.240000,1.830000}});
    cout << "Segment blue 2.240000 3.050000 2.240000 1.830000" << endl;
}

#define x_min -5.0
#define x_max  5.0
#define y_min -5.0
#define y_max  5.0
#define n_particles 250
#define trans_particles 4

int main(int argc, char** argv) {
    build_map();
    double rx=0,ry=0,r_theta=0;
    double fx=0,fy=0,f_theta=0;
    bool first = true;

    random_device rng;
    normal_distribution<double> noise_gen(0,DEV_PER_M);
    normal_distribution<double> laser_noise(0,0.001);

    cin >> rx >> ry;
    fx = rx;
    fy = ry;
    vector<Pose> particles(n_particles, {{fx,fy},r_theta});
    // vector<Pose> particles;
    // for(int i = 0; i < n_particles; i++) {
    //     double x = ((float)rng()-rng.min())/(rng.max()-rng.min()) * (x_max-x_min) + x_min;
    //     double y = ((float)rng()-rng.min())/(rng.max()-rng.min()) * (y_max-y_min) + y_min;
    //     double theta = ((float)rng()-rng.min())/(rng.max()-rng.min()) * 2*M_PI;
    //     particles.push_back({{x,y},theta});
    // }
    Particles particle_filter(particles.begin(),particles.end(),DEV_PER_M,ROBOT_WIDTH,trans_particles);
    int previous_visible = 0;
    while(true) {
        double delta_x,delta_y,delta_theta;
        // Calculate robot position, and deltas
        //
        {
            double new_x, new_y, new_theta;
            cin >> new_x >> new_y;
            new_theta = r_theta;
            delta_x = new_x-rx;
            delta_y = new_y-ry;
            if (delta_x * delta_x + delta_y * delta_y > 0.0001) {
                new_theta = atan2(delta_y,delta_x);
            }
            delta_theta = new_theta-r_theta;
            rx = new_x;
            ry = new_y;
            r_theta = new_theta;
        }

        // Calculate fake odometry
        //
        {
            double r = sqrt(delta_x * delta_x + delta_y * delta_y);
            r += noise_gen(rng)*r;
            double delta_theta_noise = delta_theta + noise_gen(rng)*delta_theta;
            f_theta += delta_theta_noise;

            double delta_x_noise = r*cos(f_theta);
            double delta_y_noise = r*sin(f_theta);
            fx += delta_x_noise;
            fy += delta_y_noise;
            particle_filter.transition({{delta_x_noise,delta_y_noise},delta_theta_noise});
        }

        // Calculate the real sonar readings
        //
        double max_range = 2.0;
        int n_points = 40;
        vector<Segment> visible_segments;
        {
            // We have n_points points going from -pi/2 to pi/2
            //
            vector<Point> observations;
            double delta_angle = (M_PI/2.0-(-M_PI/2.0)) / (n_points-1);
            Point robot = {rx,ry};
            for (int i = 0; i < n_points; i++) {
                double theta = r_theta + i*delta_angle + (-M_PI/2.0);
                Segment ray = {{rx,ry},{rx+max_range*cos(theta),ry+max_range*sin(theta)}};
                double range = max_range*2.0;
                for (vector<Segment>::iterator j = field.begin(); j != field.end(); j++) {
                    if(ray.intersects(*j)) {
                        Point intersection = ray.intersection_point(*j);
                        range = fmin(range,intersection.distance(robot));
                    }
                }
                range = fmin(range+laser_noise(rng),max_range);
                if (range < max_range-0.001) {
                    observations.push_back({range*cos(theta-r_theta),range*sin(theta-r_theta)});
                }
                ray = {{rx,ry}, {rx+range*cos(theta),ry+range*sin(theta)}};
                cout << "Segment " << (range < max_range ? "green":"red") << " " << ray.p1.x << " " << ray.p1.y << " " << ray.p2.x << " " << ray.p2.y << endl;
            }
            particle_filter.observe(field,observations);
        }

        cout << "Robot blue " << rx << " " << ry << " " << r_theta << endl;
        cout << "Robot green " << fx << " " << fy << " " << f_theta << endl;

        for(vector<Pose>::iterator p = particle_filter.particles.begin();
            p != particle_filter.particles.end();
            p++) {
            cout << "Robot red " << p->loc.x << " " << p->loc.y << " " << p->theta << endl;
        }
        // Pose mean = particle_filter.mean();
        // cout << "Robot red " << mean.loc.x << " " << mean.loc.y << " " << mean.theta << endl;
        int drawn = n_points+2+particle_filter.particles.size();
        for(int i = 0; !first && i < drawn; i++) {
            cout << "RESET " << -drawn-1 << endl;
        }
        first = false;
    }
    return 0;
}
