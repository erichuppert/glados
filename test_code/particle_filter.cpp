using namespace std;
#include "ransac_lines.h"
#include "particle_filter.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <limits>
#include <iostream>

#define N_THREADS 4

vector<Segment> transform(const Pose& delta, vector<Segment> segments) {
    for(vector<Segment>::iterator s = segments.begin(); s != segments.end(); s++) {
        *s = transform(delta, *s);
    }
    return segments;
}

// Computes scores, and reorients particles
//
void ParticleSorter::compute_scores(vector<Pose>& particles) {
    for(vector<Pose>::iterator p = particles.begin(); p != particles.end(); p++) {
        double error=0;
        bool adjusted = false;
        for (vector<Segment>::const_iterator vs = matched_vision.begin(),
                 ms = matched_world.begin();
             vs != matched_vision.end(); vs++) {
            Segment s = transform(*p, *vs);
            double angle = s.angleTo(*ms);
            angle = angle > M_PI/2.0 ? M_PI-angle : angle;
            double dist = (s.distanceTo(ms->p1) + s.distanceTo(ms->p2))*0.5;
            error += dist+(angle > M_PI/2.0 ? M_PI-angle : angle);
            // }
        }
        errors[*p] = (int) (error/PRECISION);
    }
}

bool ParticleSorter::operator()(const Pose& p1, const Pose& p2) {
    return errors[p1] < errors[p2];
}


void transition_thread(const Pose& delta
                       , vector<Pose>& new_particles
                       , mutex& mtx
                       , vector<Pose>::const_iterator& start
                       , vector<Pose>::const_iterator& end
    ) {
    
}
void Particles::transition (const Pose& delta) {
    vector<Pose> new_particles;
    if (delta.loc.abs() < PRECISION && fabs(delta.theta) < PRECISION) {
        return;
    }
    for(vector<Pose>::iterator i = particles.begin(); i != particles.end(); i++) {
        for(int j = 0; j < transition_particles; j++) {
            new_particles.push_back({{i->loc.x+delta.loc.x,i->loc.y+delta.loc.y},i->theta+delta.theta});
            double r = delta.loc.abs();
            r += noise_gen(rng)*r;
            double theta = i->theta + delta.theta +
                noise_gen(rng)*delta.theta;
            double x = i->loc.x + r*cos(theta);
            double y = i->loc.y + r*sin(theta);
            new_particles.push_back({{x,y},theta});
        }
    }
    particles = new_particles;
}

// Assumes observation points relative to origin of robot
// TODO: make multi-threaded
// TODO: mapping
// Returns the ParticleSorter which has information about the visible segments, and how they match the known map. It can be used to expand the map later on.
//
#define SEGMENT_SEPARATION 0.5
ParticleSorter Particles::observe(const vector<Segment>& map, const vector<Point>& obs) {
    // Find line segments from observations
    //
    ParticleSorter cmp;
    vector<Segment> visible_segments;
    vector<LineModel*> models = lines(obs);
    for(vector<LineModel*>::iterator m = models.begin(); m != models.end(); m++) {
        vector<Segment> model_segments = (*m)->segments(SEGMENT_SEPARATION);
        visible_segments.insert(visible_segments.end()
                                , model_segments.begin()
                                , model_segments.end());
        delete (*m); // make sure to release memory
    }

    // Find the map segments they match
    // Assumes they need to intersect with that segment,
    //  and don't intersect with others (performance justification)
    //
    Pose current_estimate = mean();
    for (vector<Segment>::iterator vs = visible_segments.begin(); vs != visible_segments.end(); vs++) {
        bool found = false;
        Segment s = transform(current_estimate, *vs);

        for (vector<Segment>::const_iterator ms = map.begin(); ms != map.end(); ms++) {
            if (s.intersects(*ms)) {
                // Found true love's match :)
                found = true;
                cmp.match(*vs,*ms);
                break;
            }
        }
        if (!found) {
            // No match :(
            cmp.single(*vs);
        }
    }

    for(vector<Pose>::iterator i = particles.begin(); i != particles.end(); i++) {
        Pose p = *i;
    }
    cmp.compute_scores(particles);
    if (cmp.matched_world.size() == 0) {
        particles = sample(particles, n_particles);
    } else {
        sort(particles.begin(),particles.end(), cmp);
        while(particles.size() != n_particles) {
            particles.pop_back();
        }
    }
    return cmp;
}

Pose Particles::mean() {
    Pose mean = {{0,0},0};
    for(vector<Pose>::iterator i = particles.begin(); i != particles.end(); i++) {
        mean = mean+(*i);
    }
    mean.loc = mean.loc*(1.0/particles.size());
    mean.theta /= particles.size();
    return mean;
}
