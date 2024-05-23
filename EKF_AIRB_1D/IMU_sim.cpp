// EVERYTHING IN THIS FILE IS NOT IMPORTANT TO CONTROL ALGORITHM
// IT IS JUST SIMULATING THE DYNAMICS OF THE ROCKET TO TEST/IMPROVE THE CONTROL

#ifndef IMU_SIM_H
#define IMU_SIM_H


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>


#define THRUST_TIME 2 // in [s]
#define THRUST_FORCE 2000 // in [N]
#define GRAV_CONST 9.81
#define RHO 1.293
#define MASS 5
#define MEASUREMENT_VAR 5



// this structure contains all data that sensors would record
typedef struct IMU {
    float a[3]; // 3 axis accel [x, y, z]
    float w[3]; // 3 axis rotation [roll, pitch, yaw] assuming coordinate transforms already took place
    float s; // baro relationship

    // dont touch these in algorithm, they are for modelling
    float a_real; // real a in z-dir
    float v_real; // real v in z-dir
    float s_real; // real h in z-dir
    float dt; // time step

} IMU;

float thrust_curve(float t) {
    float F;
    if (t > THRUST_TIME) {
        F = 0;
    } else {
        F = THRUST_FORCE;
    }
    return F;
}


// (TODO) thrust curve modelled off real data is better
float accel_curve(float t, float cd, float A, float v) {
    float a;
    if (t > THRUST_TIME) {
        // abs(v) accounts for the direction of drag opposite to vel
        a = -GRAV_CONST - (v/fabs(v))*(RHO * cd * A * v*v) / (2*MASS);
    } else {
        a = -GRAV_CONST - (RHO * cd * A * v*v) / (2*MASS) + THRUST_FORCE/MASS;
    }
    return a;
}



float rand_normal(float meas_var) {
    float u1 = rand() / (float)RAND_MAX;
    float u2 = rand() / (float)RAND_MAX;
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2) * sqrt(meas_var);
}


IMU* generate_model_data(IMU* data_prev, float t, float A, float cd) {

    float a_new = accel_curve(t, cd, A, data_prev->v_real);
    float v_new = data_prev->v_real + a_new * data_prev->dt;
    float h_new = data_prev->s_real + v_new * data_prev->dt;
    
    // normally distributed noise -> estimate MEASUREMENT_VAR in Kalman filter
    float a_noisy = a_new + rand_normal(MEASUREMENT_VAR);
    float h_noisy = h_new + rand_normal(MEASUREMENT_VAR);

    IMU *data_new = (IMU*) malloc(sizeof(IMU));
    data_new->a[0] = a_noisy;
    data_new->s = h_noisy;
    data_new->a_real = a_new;
    data_new->v_real = v_new;
    data_new->s_real = h_new;

    data_new->dt = data_prev->dt;
    return data_new;
}


#endif