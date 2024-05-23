#ifndef KF_H
#define KF_H

#include <iostream>
#include "../thirdparty/eigen/Eigen/Dense"


class KF {
    private:
        // for propagate_dynamics
        void propagate_state();
        void propagate_cov();

        // for rec_least_squares
        void update_gain();
        void update_exp_cov();
        void update_state_prediction(float meas);

   
    public:
        Eigen::Matrix<float, 3, 1> x;
        Eigen::Matrix<float, 1, 1> y;
        Eigen::Matrix<float, 3, 3> A; // A = e^(A_C * dt)
        Eigen::Matrix<float, 1, 3> C;
        Eigen::Matrix<float, 3, 1> K;
        Eigen::Matrix<float, 1, 1> R;
        Eigen::Matrix<float, 3, 3> P;
        Eigen::Matrix<float, 3, 3> I;

        KF(float dt, float init_a, float init_v, float init_s, float meas_var);

        // previous posteriori to current priory
        void propagate_dynamics();
        // current priory to current posteriori, AKA recursive least squares implementation
        void rec_least_squares(float meas);
        
        void retrieve_state(float &pos, float &vel, float &accel);
};


KF::KF(float dt, float init_a, float init_v, float init_s, float meas_var) {
    x << init_s, init_v, init_a;
    C << 1, 0, 0;
    A << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1;
    R << meas_var;
    P = Eigen::Matrix3f::Identity() * 100;  //Because assuming not good knowledge of this state, if perfect P = 0 matrix
    I = Eigen::Matrix3f::Identity();
}

void KF::retrieve_state(float &pos, float &vel, float &accel) {
    pos = x(0, 0);
    vel = x(1, 0);
    accel = x(2, 0);
}

void KF::propagate_dynamics() {
    propagate_state();
    propagate_cov();
}

void KF::rec_least_squares(float meas) {
    update_gain();
    update_exp_cov();
    update_state_prediction(meas);
}


void KF::propagate_state() {
    // x was the prev posteriori but will now be the current priori
    // there would be a +BU term here if control is applied!!
    // this model assumes A is unchanging
    x = A * x;
}

void KF::propagate_cov() {
    // P was the prev posteriori but will now be the current priori
    // here you would add process noise but assumed zero
    P = A * P * A.transpose();
}


// These last three are copied from RLS implementation
void KF::update_gain() {
    K = P * C.transpose() * (R + C * P * C.transpose()).inverse();
}

void KF::update_exp_cov() {
    P = (I - K * C) * P;
}

void KF::update_state_prediction(float meas) {
    y << meas;
    x = x + K * (y - C * x);
}

#endif