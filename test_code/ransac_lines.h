#ifndef RANSAC_LINES_H
#define RANSAC_LINES_H

#include "ransac.h"
#include <cmath>

#define PRECISION 0.000001
struct Point {
public:
    double x;
    double y;

    bool operator<(const Point& rhs) const {
        int my_x = x*(1./PRECISION);
        int my_y = y*(1./PRECISION);
        int rhs_x = rhs.x*(1./PRECISION);
        int rhs_y = rhs.y*(1./PRECISION);
        return (my_x < rhs_x) || (my_x == rhs_x && my_y < rhs_y);
    }

    bool operator==(const Point& rhs) const {
        int my_x = x*(1./PRECISION);
        int my_y = y*(1./PRECISION);
        int rhs_x = rhs.x*(1./PRECISION);
        int rhs_y = rhs.y*(1./PRECISION);
        return my_x == rhs_x && my_y == rhs_y;
    }

    inline Point operator-(const Point& rhs) const {
        return {x: x-rhs.x, y: y-rhs.y};
    }
    inline Point operator+(const Point& rhs) const {
        return {rhs.x+x, rhs.y+y};
    }
    inline Point operator*(const double& c) const {
        return {x*c,y*c};
    }

    inline double abs() const {
        return sqrt(x*x+y*y);
    }

    inline double dot(const Point& other) const {
        return x*other.x + y*other.y;
    }

    inline double distance(const Point& other) const {
        return ((*this)-other).abs();
    }
};

class Segment;
class LineModel: public Model<Point> {
private:
    // Information needed
    //
    int n;
    double x_sum;
    double y_sum;
    double x2_sum;
    double y2_sum;
    double xy_sum;
    double y_mean;
public:
    // y = mx + b
    //
    double m;
    double b;

    // Unit vector:
    // u0*x + u1*y + u2 = 0
    //
    double u0;
    double u1;
    double u2;

    std::vector<Point> data;

    LineModel(const std::vector<Point>& data);
    void add_point(const Point& point);
    double error(const Point& point);
    double error(const std::vector<Point>& data);
    std::vector<Segment> segments(double threshold);
};

extern std::function<Model<Point>*(std::vector<Point>&)> model_factory;
std::vector<LineModel*> lines(const std::vector<Point>& data);
#endif

struct Segment {
private:
    int orientation(const Point& p, const Point& q, const Point& r) const;
public:
    Point p1;
    Point p2;

    double distanceTo(const Point& p) const;
    inline bool operator<(const Segment& rhs) const {
        return (p1 < rhs.p1) || (p1 == rhs.p1 && p2 < rhs.p2);
    }
    bool intersects(const Segment& other) const;
    Point intersection_point(const Segment& other) const;
    inline bool onSegment(const Point& p) const {
        return
            orientation(p,p1,p2) == 0 &&
            p.x <= fmax(p1.x,p2.x) && p.x >= fmin(p1.x,p2.x) &&
            p.y <= fmax(p1.y,p2.y) && p.y >= fmin(p1.y,p2.y);
    }
    inline double angleTo(const LineModel& lm) const {
        Point v1 = p2-p1;
        Point v2 = (Point){p2.x,lm.m*p2.x} - (Point){p1.x,lm.m*p1.x};
        if (v1.abs() * v2.abs() == 0) {
            return M_PI/2.0;
        }
        return acos(v1.dot(v2)/(v1.abs() * v2.abs()));
    }
    inline double angleTo(const Segment& s) const {
        Point v1 = p2-p1;
        Point v2 = s.p2-s.p1;
        if (v1.abs() * v2.abs() == 0) {
            return M_PI/2.0;
        }
        return acos(v1.dot(v2)/(v1.abs() * v2.abs()));
    }
};
