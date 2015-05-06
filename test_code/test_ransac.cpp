using namespace std;
#include "ransac.h"
#include "ransac_lines.h"
#include <iostream>
#include <random>
#include <cmath>
#include <functional>

#define ERROR_THRESHOLD 0.01
void test_sampling(int N) {
    vector<int> original(10);
    for(int i=0; i < 10; i++) {
        original[i] = i;
    }

    vector<int> count(10,0);
    cout << N << " samples" << endl;
    for(int i = 0; i < N; i++) {
        vector<int> samples = sample(original, 1);
        count[samples[0]]++;
    }
    double error = 0;
    for(int i = 0; i < 10; i++) {
        error += fabs(count[i]/((float)N) - 0.1);
    }
    cout << "Error: " << error << ", " << ((error>ERROR_THRESHOLD)?"Sampling biased":"Sampling unbiased") << endl;
}

#define LINE_ERROR_THRESHOLD 2.0
inline double line_error(double m_expected, double b_expected, LineModel* model) {
    return fabs(model->m-m_expected) + fabs(model->b - b_expected);
}
vector<Point> sample_line(int N, bool noise, double m, double b) {
    random_device rng;
    vector<Point> v(N);
    normal_distribution<double> noise_gen(0,0.01);
    for(int i = 0; i < N; i++) {
        double x = (rng()%10000)/100.;
        double y_noise = noise?noise_gen(rng):0;
        v[i] = {x: x,y: m*x+b+y_noise};
    }
    return v;
}

void test_line_fitting(int N) {
    double m = 10.0;
    double b = 5.0;
    vector<Point> l = sample_line(N,true,m,b);
    LineModel model(l);
    double error = line_error(m,b,&model);
    cout << "Error: " << error << ", " <<
        ((error>LINE_ERROR_THRESHOLD)?"Fitting wrong":"Fitting correct") << endl;
}

void test_ransac_line(int N) {
    double m = 10.0;
    double b = 5.0;
    vector<Point> l = sample_line(N,true,m,b);
    LineModel* model = (LineModel*)ransac(l, model_factory, 10, 100, 0.001);
    double error = line_error(m,b,model);
    cout << "Error: " << error << ", " <<
        ((error>LINE_ERROR_THRESHOLD)?"RANSAC fitting wrong":"RANSAC fitting correct") << endl;
    delete model;
}

void test_ransac_two_line(int N) {
    double m1 = 10.0;
    double b1 = 5.0;
    double m2 = 20.0;
    double b2 = 5.0;
    vector<Point> l1 = sample_line(N,true,m1,b1);
    vector<Point> l2 = sample_line(N,true,m2,b2);
    set<Point> data(l1.begin(),l1.end());
    data.insert(l2.begin(), l2.end());

    int min_points = 4;
    // at least 99% chance of hitting the line
    // Geometric with probability of success p = (1/2)^(min_points)
    // CDF: 1-(1-p)^(iterations)
    // So iterations needs to solve:
    // .99 = 1-(1-p)^(iterations) -> ln(.01)/ln(1-p) = iterations
    // -4.605170186 / (~ (1-p)-1) = iterations -> iterations ~ 4.605 * 2^(min_points)
    //
    int iterations = 4.605*powl(2,min_points);

    // First line
    //
    vector<Point> data_vector(data.begin(), data.end());

    LineModel* model1 = (LineModel*)ransac(data_vector, model_factory, min_points, iterations, 0.01);
    if (model1 == nullptr) {
        cout << "BAD" << endl;
        return;
    }
    for(vector<Point>::iterator i = model1->data.begin(); i != model1->data.end(); i++) {
        data.erase(*i);
    }

    // Second line
    //
    data_vector = vector<Point>(data.begin(), data.end());
    LineModel* model2 = (LineModel*)ransac(data_vector, model_factory, min_points, iterations, 0.01);
    if (model2 == nullptr) {
        cout << "BAD" << endl;
        return;
    }

    double error1_1 = line_error(m1,b1,model1);
    double error1_2 = line_error(m2,b2,model1);
    double error1 = fmin(error1_1, error1_2);
    double error2 = error1_1 > error1_2 ? line_error(m1,b1,model2):line_error(m2,b2,model2);

    cout << ((error1 < LINE_ERROR_THRESHOLD && error2 < LINE_ERROR_THRESHOLD)?"GOOD":"BAD") << endl;

    delete model1;
    delete model2;
}

vector<Point> random_points(int N) {
    random_device rng;
    vector<Point> v(N);
    for(int i = 0; i < N; i++) {
        double x = (rng()%10000)/100.;
        double y = (rng()%10000)/100.;
        v[i] = {x: x,y: y};
    }
    return v;
}

void test_bic_lines(int N) {
    random_device rng;
    double m1 = (rng()%10000)/100.0;
    double b1 = (rng()%10000)/100.0;
    double m2 = (rng()%10000)/100.0;
    double b2 = (rng()%10000)/100.0;
    vector<Point> l1 = sample_line(N,true,m1,b1);
    vector<Point> l2 = sample_line(N,true,m2,b2);
    vector<Point> rand_data = random_points(N/4);
    vector<Point> data(l1);
    data.insert(data.end(), l2.begin(), l2.end());
    data.insert(data.end(), rand_data.begin(), rand_data.end());

    vector<LineModel*> models = lines(data);
    if (models.size() != 2) {
        cout << "BAD NUMB OF LINES, " << models.size() << endl;
        return;
    }
    double error1_1 = line_error(m1,b1,models[0]);
    double error1_2 = line_error(m2,b2,models[0]);
    double error1 = fmin(error1_1, error1_2);
    double error2 = error1_1 > error1_2 ? line_error(m1,b1,models[1]):line_error(m2,b2,models[1]);

    cout << ((error1 < LINE_ERROR_THRESHOLD && error2 < LINE_ERROR_THRESHOLD)?"GOOD":"BAD") << endl;

    delete models[0];
    delete models[1];
}

int main(int argc, char** argv) {
    int N = 300;
    int test = 0;
    if (argc >= 3) {
        test = stoi(argv[1]);
        N = stoi(argv[2]);
    } else {
        cout << "INVALID ARGUMENTS, Should be TEST_NUM DATA_POINTS" << endl;
        return 1;
    }
    switch(test) {
    case 0:
        // Test Sampling
        //
        test_sampling(N);
        break;
    case 1:
        // Test line fitting
        //
        test_line_fitting(N);
        break;
    case 2:
        // Test line fitting with RANSAC
        //
        test_ransac_line(N);
        break;
    case 3:
        // Test dual line fitting with RANSAC
        //
        test_ransac_two_line(N);
        break;
    case 4:
        // Test BIC dual line fitting with RANSAC
        //
        test_bic_lines(N);
        break;
    }
}
