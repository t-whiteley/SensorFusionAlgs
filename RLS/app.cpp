#include "RLS.cpp"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


#define DT 0.01


float rand_normal(float meas_var) {
    float u1 = rand() / (float)RAND_MAX;
    float u2 = rand() / (float)RAND_MAX;
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2) * sqrt(meas_var);
}


int main() {
    // not sure what the use in this model is, takes in so much data just to find the initial state of the system which its no longer in
    srand(time(NULL));
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,a_real,a_filt\n");

    float t = 0;
    float v_init = 20, s_init = 10;
    float a_real = 5, v_real = v_init, s_real = s_init;
    float a_filt, v_filt, s_filt, s_noisy;


    // here I am providing the initial guess for x, I am giving it a terrible first guess of 0, 0, 0
    // x is a state vector, which includes the initial height, initial velocity and constant acceleration. not the ongoing values
    RLS rls(DT, 0, 0, 0, 0.5);

    for (int i = 0; i < 500; i++) {
        t += DT;
        v_real += DT * a_real;
        s_real += DT * v_real;
        s_noisy = s_real + rand_normal(0.5);

        rls.predict_state(s_noisy);
        rls.update_model();

        s_filt = rls.x(0,0);
        v_filt = rls.x(1,0);
        a_filt = rls.x(2,0);
        
        fprintf(csv, "%.4f,%.4f,%.4f\n", t, a_real, a_filt);
        // fprintf(csv, "%.4f,%.4f,%.4f\n", t, v_init, v_filt);
        // fprintf(csv, "%.4f,%.4f,%.4f\n", t, s_init, s_filt);
    }

    fclose(csv);
}