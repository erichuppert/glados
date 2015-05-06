#ifndef RANSAC_H
#define RANSAC_H

#include <random>
#include <thread>
#include <mutex>
#include <set>
#include <limits>
#include <functional>
#include <vector>
#include <algorithm>
#define N_THREADS 4

template<typename point>
class Model {
public:
    virtual double error(const point& p) = 0;
    virtual double error(const std::vector<point>&) = 0;
    virtual void add_point(const point& p) = 0;
};

template<typename T>
std::vector<T> sample(std::vector<T> points, int n_samples) {
    if (points.size() <= n_samples) {
        return points;
    }

    std::vector<T> samples;
    samples.reserve(n_samples);
    std::random_device rng;

    for(int i = 0; i < n_samples; i++) {
        int index = rng()%(points.size()-i); // Not true random, but close enough
        samples.push_back(points[index]);
        points[index] = points[points.size()-i-1];
    }

    return samples;
}

template<typename point>
void ransac_thread(std::vector<point>& data
                   , std::function<Model<point>*(std::vector<point>&)> model_factory
                   , int min_points
                   , int iterations
                   , double threshold
                   , std::mutex& mtx
                   , Model<point>*& best_model
                   , double& best_error
    ) {

    for (int i = 0; i < iterations; i++) {
        std::vector<point> samples = sample(data, min_points);
        std::set<point> samples_set(samples.begin(), samples.end());
        std::vector<point> new_inliers;
        Model<point>* model = model_factory(samples);
        for (typename std::vector<point>::iterator j = data.begin(); j != data.end(); j++) {
            if (samples_set.find(*j) != samples_set.end()) {
                continue;
            }
            if (model->error(*j) < threshold) {
                new_inliers.push_back(*j);
            }
        }

        double error = new_inliers.size();//model->error(new_inliers);

        mtx.lock();
        if(error > best_error && new_inliers.size() >= min_points) {
            delete best_model;
            best_model = model;
            for(typename std::vector<point>::iterator j = new_inliers.begin(); j != new_inliers.end(); j++) {
                best_model->add_point(*j);
            }
            best_error = error;
        } else {
            delete model;
        }
        mtx.unlock();
    }
}

template<typename point>
Model<point>* ransac(std::vector<point> data,
                     std::function<Model<point>*(std::vector<point>&)> model_factory,
                     int min_points,
                     int iterations,
                     double threshold) {

    Model<point>* best_model = nullptr;
    // Do nothing if we don't have enough data points
    //
    if (data.size() < min_points) {
        return best_model;
    }

    double best_error = std::numeric_limits<double>::min();
    std::mutex mtx;
    std::vector<std::thread> threads;
    for(int i = 0; i < N_THREADS; i++) {
        threads.push_back(std::thread(ransac_thread<point>,
                                      std::ref(data)
                                      , model_factory
                                      , min_points
                                      , iterations/N_THREADS
                                      , threshold
                                      , std::ref(mtx)
                                      , std::ref(best_model)
                                      , std::ref(best_error)
                              ));
    }

    for(int i = 0; i < N_THREADS; i++) {
        threads[i].join();
    }

    return best_model;
}

#endif
