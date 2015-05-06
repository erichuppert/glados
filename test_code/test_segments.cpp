using namespace std;
#include "ransac_lines.h"
#include <iostream>
#include <string>
#include <cassert>
#include <vector>
#include <cmath>

void test_intersect() {
    Segment s1 = {{0,0},{10,0}};
    Segment s2 = {{5,-10}, {5,10}};
    Segment s3 = {{-10,5}, {10,5}};
    assert(s1.intersects(s1));
    assert(s1.intersects(s2));
    assert(!s1.intersects(s3));
    assert(s2.intersects(s1));
    assert(s2.intersects(s2));
    assert(s2.intersects(s3));
    assert(!s3.intersects(s1));
    assert(s3.intersects(s2));
    assert(s3.intersects(s3));
}

void test_angle() {
    vector<Point> line_data;
    line_data.push_back({0,0});
    line_data.push_back({1,0});
    LineModel lm(line_data);
    Segment s3 = {{0,0},{1,0}};
    Segment s1 = {{-1,-1},{-1,1}};
    Segment s2 = {{-1,-1},{1,1}};

    assert(fabs(s1.angleTo(lm)-M_PI/2.0) < PRECISION);
    assert(fabs(s2.angleTo(lm)-M_PI/4.0) < PRECISION);
    assert(fabs(s1.angleTo(s3)-M_PI/2.0) < PRECISION);
    assert(fabs(s2.angleTo(s3)-M_PI/4.0) < PRECISION);
}

int main(int argc, char** argv) {
    int test = 0;
    if (argc >= 2) {
        test = stoi(argv[1]);
    } else {
        cout << "INVALID ARGUMENTS, Should be TEST_NUM" << endl;
        return 1;
    }
    switch(test) {
    case 0:
        // Test Intersection
        //
        test_intersect();
        break;
    case 1:
        // Test Angle to line
        //
        test_angle();
        break;
    }
}
