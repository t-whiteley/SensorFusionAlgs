#ifndef RLS_H
#define RLS_H

#include <iostream>
#include "../thirdparty/eigen/Eigen/Dense"


class RLS {
    private:
        void update_exp_cov();
        void update_gain();
   
    public:
        Eigen::Matrix<float, 3, 1> x;
        Eigen::Matrix<float, 1, 1> y;
        Eigen::Matrix<float, 3, 3> P;
        Eigen::Matrix<float, 3, 1> K;
        Eigen::Matrix<float, 1, 1> R;
        Eigen::Matrix<float, 1, 3> C;
        Eigen::Matrix<float, 3, 3> I;
        float delt;
        float step;

        RLS(float dt, float init_a, float init_v, float init_s, float meas_var);
        void update_model();
        void predict_state(float meas);
};

RLS::RLS(float dt, float init_a, float init_v, float init_s, float meas_var) {
    x << init_s, init_v, init_a;
    P = Eigen::Matrix3f::Identity();
    I = Eigen::Matrix3f::Identity();
    R << meas_var;
    delt = dt;
    step = 1;

    update_model();
}

void RLS::update_model() {
    C << 1, step * delt, step * step * delt * delt / 2;
    update_gain();
    update_exp_cov();
}

void RLS::update_gain() {
    K = P * C.transpose() * (R + C * P * C.transpose()).inverse();
}

void RLS::update_exp_cov() {
    P = (I - K * C) * P;
}

void RLS::predict_state(float meas) {
    y << meas;
    x = x + K * (y - C * x);
    step++;
}

#endif