using namespace std;
#include "ransac_lines.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cassert>

// http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// for orientation, and intersects
//

double Segment::distanceTo(const Point& p) const {
    // Return minimum distance between line segment, and point p
    Point vec = p2-p1;
    const float l2 = vec.x*vec.x + vec.y*vec.y;  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return p1.distance(p);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    const float t = (p-p1).dot(p2-p1)/l2;
    if (t < 0.0) return p1.distance(p);       // Beyond the 'v' end of the segment
    else if (t > 1.0) return p2.distance(p);  // Beyond the 'w' end of the segment
    const Point projection = p1 + vec*t;  // Projection falls on the segment
    return projection.distance(p);
}

int Segment::orientation(const Point& p, const Point& q, const Point& r) const {
    // See 10th slides from following link for derivation of the formula
    // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    double val = (q.y - p.y) * (r.x - q.x) -
        (q.x - p.x) * (r.y - q.y);

    if (fabs(val) < PRECISION) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool Segment::intersects(const Segment& other) const {
    // Find the four orientations needed for general and
    // special cases

    int o1 = orientation(p1, p2, other.p1);
    int o2 = orientation(p1, p2, other.p2);
    int o3 = orientation(other.p1, other.p2, p1);
    int o4 = orientation(other.p1, other.p2, p2);

    // General case
    if (o1 != o2 && o3 != o4) {
        return true;
    }
    // Special Cases (colinear)
    //
    if (o1 == 0 && onSegment(other.p1)) return true;
    if (o2 == 0 && onSegment(other.p2)) return true;
    if (o3 == 0 && other.onSegment(p1)) return true;
    if (o4 == 0 && other.onSegment(p2)) return true;

    return false; // Doesn't fall in any of the above cases
}

Point Segment::intersection_point(const Segment& other) const {
    // Assumes intersection exists
    // Returns the intersection point
    //
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double xx1 = other.p1.x;
    double yy1 = other.p1.y;
    double xx2 = other.p2.x;
    double yy2 = other.p2.y;

    double top = (yy1-y1)*(x2-x1) - (xx1-x1)*(y2-y1);
    double bottom = (xx2-xx1)*(y2-y1) - (yy2-yy1)*(x2-x1);
    double alpha_2 = top/bottom;

    if (fabs(bottom) < PRECISION) { // colinear
        return onSegment(other.p1) ? other.p1 : other.p2;
    }

    return other.p2 * alpha_2 + other.p1 * (1-alpha_2);
}

LineModel::LineModel(const vector<Point>& data):
    n(0),
    x_sum(0),
    y_sum(0),
    x2_sum(0),
    y2_sum(0),
    xy_sum(0),
    data(data)
{
    for (vector<Point>::const_iterator i = data.cbegin(); i != data.cend(); i++) {
        n++;
        x_sum += i->x;
        y_sum += i->y;
        x2_sum += i->x*i->x;
        y2_sum += i->y*i->y;
        xy_sum += i->x*i->y;
    }
    double x_mean = x_sum/n;
    double y_mean = y_sum/n;
    m = (xy_sum - x_sum*y_mean) / (x2_sum - x_sum * x_mean);
    b = y_mean - m*x_mean;

    // Find unit vector
    //
    double d = x2_sum*y2_sum - xy_sum*xy_sum; // Assume it's not zero - dangerous
    u0 = (x_sum * y2_sum - y_sum * xy_sum)/d;
    u1 = (y_sum * x2_sum - x_sum * xy_sum)/d;
    double length = sqrt(u0*u0 + u1*u1);
    u0 /= length;
    u1 /= length;
    u2 = -1./length;
}

void LineModel::add_point(const Point& point) {
    n++;
    x_sum += point.x;
    y_sum += point.y;
    x2_sum += point.x*point.x;
    y2_sum += point.y*point.y;
    xy_sum += point.x*point.y;
    data.push_back(point);

    double x_mean = x_sum/n;
    y_mean = y_sum/n;
    assert(x2_sum - x_sum*x_mean != 0); // Monitor special cases
    m = (xy_sum - x_sum*y_mean) / (x2_sum - x_sum * x_mean);
    b = y_mean - m*x_mean;

    // Find unit vector
    //
    double d = x2_sum*y2_sum - xy_sum*xy_sum;
    assert(d != 0); // Monitor possible special lines
    u0 = (x_sum * y2_sum - y_sum * xy_sum)/d;
    u1 = (y_sum * x2_sum - x_sum * xy_sum)/d;
    double length = sqrt(u0*u0 + u1*u1);
    u0 /= length;
    u1 /= length;
    u2 = -1./length;
}

double LineModel::error(const Point& point) {
    double residual = point.y - (m*point.x+b);
    return residual*residual;
    // return fabs(u0*point.x + u1*point.y + u2);
}

double LineModel::error(const vector<Point>& data) {
    double data_error = 0;
    double dy_sum = 0;
    double dy2_sum = 0;
    double fi_sum = 0;
    double fi2_sum = 0;
    for(vector<Point>::const_iterator i = data.begin(); i != data.end(); i++) {
        dy_sum += i->y;
        dy2_sum += i->y*i->y;
        double fi = m*i->x + b;
        fi_sum += fi;
        fi2_sum += fi*fi;
    }
    double tot = dy2_sum + dy_sum * y_mean + y_mean*y_mean * data.size();
    double res = dy2_sum + dy_sum * fi_sum/data.size() + fi2_sum;
    double r2 = 1.0-res/tot;
    return fabs(r2-1);
}

vector<Segment> LineModel::segments(double threshold) {
    vector<Segment> segments;
    sort(data.begin(),data.end()); // Sorted by x, then y
    int s = -1;
    for (vector<Point>::iterator i = data.begin(); i != data.end(); i++) {
        Point projection = {i->x,i->x*m + b};
        if(s == -1 || projection.distance(segments[s].p2) > threshold) {
            segments.push_back({projection,projection});
            s++;
        } else {
            segments[s].p2 = projection;
        }
    }
    return segments;
}

inline Model<Point>* _model_factory(vector<Point>& data) {
    return new LineModel(data);
}

function<Model<Point>*(vector<Point>&)> model_factory = _model_factory;

inline double normal_prob(double mean, double std_dev, double x) {
    return exp(- (x-mean)*(x-mean) / (2*std_dev*std_dev)) / (std_dev*sqrt(2*M_PI));
}

#define MAX_LINES 5
#define NOISE_STD_DEV 0.001
#define NOISE_MEAN 0
#define FRAC_QUIT 0.9 // 90% of points covered, we're done.
vector<LineModel*> lines(const vector<Point>& data) {
    vector<LineModel*> models;
    double bic = numeric_limits<double>::max();
    for(int n_lines = 1; n_lines <= MAX_LINES; n_lines++) {
        vector<LineModel*> new_models;
        set<Point> data_set(data.begin(), data.end());

        // At least 99% chance of choosing min_points all in the same line.
        // The factor of 40 is arbitrary
        //
        int min_points = fmax(data.size()/(40*n_lines), 2);
        int iterations = 4.605*powl(n_lines,min_points);

        // Generate the line models
        //
        for(int line = 0; line < n_lines; line++) {
            vector<Point> data_vector(data_set.begin(), data_set.end());
            LineModel* model = (LineModel*)ransac(data_vector, model_factory,
                                                  min_points, iterations,
                                                  (NOISE_STD_DEV*2)*(NOISE_STD_DEV*2));
            if (model == nullptr) {
                if (models.size() != 0) {
                    return models;
                } else {
                    continue;
                }
            }
            new_models.push_back(model);

            for (vector<Point>::iterator p = model->data.begin(); p != model->data.end(); p++) {
                data_set.erase(*p);
            }
        }

        // Do we cover a significant amount of points?
        //
        if ((1.0-FRAC_QUIT) * data.size() > data_set.size()) {
            return new_models;
        }

        // Find the BIC score
        // Assume gaussian noise
        //
        double prob = 1.0;
        // Remaining points, find best line
        //
        for(set<Point>::iterator p = data_set.begin(); p != data_set.end(); p++) {
            double max_prob = numeric_limits<double>::max();
            for(vector<LineModel*>::iterator model = new_models.begin(); model != new_models.end(); model++) {
                max_prob = fmax(max_prob,
                                normal_prob(NOISE_MEAN,NOISE_STD_DEV,
                                            sqrt((*model)->error(*p))));
            }
            prob *= max_prob;
        }
        // Find the likelihoods of the models' points
        //
        for(vector<LineModel*>::iterator model = new_models.begin(); model != new_models.end(); model++) {
            for(vector<Point>::iterator p = (*model)->data.begin(); p != (*model)->data.end(); p++) {
                prob *= normal_prob(NOISE_MEAN, NOISE_STD_DEV, sqrt((*model)->error(*p)));
            }
        }
        double new_bic = -2 * log(prob) + n_lines*2 * log(data.size());
        if (new_bic > bic) {
            break;
        }
        models = new_models;
        bic = new_bic;
    }
    return models;
}
