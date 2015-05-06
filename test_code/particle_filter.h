#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <set>
#include <map>
#include <iterator>
#include <random>
#include <iostream>

struct Pose {
    Point loc;
    double theta;

    inline bool operator<(const Pose& rhs) const {
        return loc < rhs.loc || (rhs.loc < loc && theta < rhs.theta);
    }
    inline Pose operator-(const Pose& rhs) const {
        return {loc-rhs.loc, theta-rhs.theta};
    }
    inline Pose operator+(const Pose& rhs) const {
        return {loc+rhs.loc, theta+rhs.theta};
    }
};
inline Point transform(const Pose& delta, const Point& original) {
    // Rotate
    double x = original.x * cos(delta.theta) - original.y * sin(delta.theta);
    double y = original.x * sin(delta.theta) + original.y * cos(delta.theta);
    // Displace
    x += delta.loc.x;
    y += delta.loc.y;
    return {x,y};
}
inline Segment transform(const Pose& delta, const Segment& s) {
    return {transform(delta,s.p1),transform(delta,s.p2)};
}

// Used to sort particles according to error
//
class ParticleSorter {
public:
    std::vector<Segment> matched_vision;
    std::vector<Segment> matched_world;
    std::vector<Segment> singles;
    std::map<Pose,int> errors;

    inline void match(const Segment& visible, const Segment& actual) {
        matched_vision.push_back(visible);
        matched_world.push_back(actual);
    }
    inline void single(const Segment& visible) {
        singles.push_back(visible);
    }
    void compute_scores(std::vector<Pose>& particles);
    bool operator()(const Pose& p1, const Pose& p2);
};

class Particles {
public:
    std::random_device rng;
    std::normal_distribution<double> noise_gen;
    std::vector<Pose> particles;
    double rotation_diameter;
    int transition_particles;
    int n_particles;
    template<class Iter>
    Particles(Iter begin,
              Iter end,
              double dev_per_m,
              double rotation_diameter,
              int transition_particles
        )
        : particles(begin,end)
        , noise_gen(0,dev_per_m)
        , rotation_diameter(rotation_diameter)
        , transition_particles(transition_particles)
        , n_particles(particles.size())
    {}

    void transition(const Pose& delta);
    ParticleSorter observe(const std::vector<Segment>& map, const std::vector<Point>& obs);
    Pose mean();
};

#endif
