#ifndef KF_H
#define KF_H

#include <iostream>
#include "IMU_sim.cpp"
#include "../thirdparty/eigen/Eigen/Dense"

class KF {
    private:
        // for predict -> propagate dynamics
        void gen_nonlinear_f();
        void update_jac_f();
        void propagate_state();
        void propagate_cov();

        // for update -> rec least squares
        void update_gain();
        void update_exp_cov();
        void update_state_prediction(float meas_a, float meas_h);

   
    public:
        Eigen::Matrix<float, 3, 1> x;
        Eigen::Matrix<float, 2, 1> y;

        Eigen::Matrix<float, 3, 1> f;
        Eigen::Matrix<float, 3, 3> F;
        float mu;
        float m;
        float g;
        float dt;
        float t;

        Eigen::Matrix<float, 2, 3> C;
        Eigen::Matrix<float, 3, 2> K;
        Eigen::Matrix<float, 2, 2> R;
        Eigen::Matrix<float, 3, 3> P;
        Eigen::Matrix<float, 3, 3> Q;
        Eigen::Matrix<float, 3, 3> I;

        KF(float delt, float init_a, float init_v, float init_s, float meas_var_a, float meas_var_h, float process_var);

        // previous posteriori to current priory
        void predict(float rho, float cd, float A, float time, float mass);
        // current priory to current posteriori, AKA recursive least squares implementation
        void update(float meas_a, float meas_h);
        
        void retrieve_state(float &pos, float &vel, float &accel);
};


KF::KF(float delt, float init_a, float init_v, float init_s, float meas_var_a, float meas_var_h, float process_var) {
    x << init_s, init_v, init_a;
    C << 1, 0, 0, 0, 0, 1;
    R << meas_var_h, 0, 0, meas_var_a;
    P = Eigen::Matrix3f::Identity() * 100;  //Because assuming not good knowledge of this state, if perfect P = 0 matrix
    I = Eigen::Matrix3f::Identity();
    Q << process_var, 0, 0, 0, process_var, 0, 0, 0, process_var;
    dt = delt;
    g = 9.81;
}

void KF::retrieve_state(float &pos, float &vel, float &accel) {
    pos = x(0, 0);
    vel = x(1, 0);
    accel = x(2, 0);
}

void KF::predict(float rho, float cd, float A, float time, float mass) {
    t = time;
    m = mass;
    mu = 0.5 * rho * cd * A;
    gen_nonlinear_f();
    update_jac_f();
    propagate_state();
    propagate_cov();
}

void KF::update(float meas_a, float meas_h) {
    update_gain();
    update_exp_cov();
    update_state_prediction(meas_a, meas_h);
}


// take my prev posteriori and find my current priori
void KF::gen_nonlinear_f() {
    f <<  x(0,0) + dt * x(1,0),
            x(1,0) + dt / m * (thrust_curve(t) - m*g - mu*x(1,0)*x(1,0)),
            x(2,0) - dt / m * (2*mu*x(1,0)*x(2,0));
}


void KF::update_jac_f() {
    // F << 1, dt, 0, 0, 1, dt, 0, -2*mu*dt*x(2,0), 1-2*dt*mu*x(1,0);
    F << 1, dt, 0,
        0, 1 - dt/m * (2*mu*x(1,0)), 0,
        0, -dt / m * 2*mu*x(2,0), 1 - dt/m*2*mu*x(1,0);
}



void KF::propagate_state() {
    // x was the prev posteriori but will now be the current priori
    // there would be a +BU term here if control is applied!!
    // this model assumes A is unchanging
    // x = A * x;
    x = f;
}

void KF::propagate_cov() {
    // P was the prev posteriori but will now be the current priori
    // P = A * P * A.transpose() + Q;
    P = F * P * F.transpose() + Q;
}


// These last three are copied from RLS implementation
void KF::update_gain() {
    K = P * C.transpose() * (R + C * P * C.transpose()).inverse();
}

void KF::update_exp_cov() {
    // P = (I - K * C) * P; // simplified eq not as reliable supposedly
    P = (I - K * C) * P * (I - K * C).transpose() + K * R * K.transpose();
}

void KF::update_state_prediction(float meas_a, float meas_h) {
    y << meas_h, meas_a;
    x = x + K * (y - C * x);
}

#endif