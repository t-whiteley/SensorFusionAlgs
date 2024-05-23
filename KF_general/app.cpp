#include "KF.cpp"

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
    srand(time(NULL));
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,a_real,a_filt,v_real,v_filt,s_real,s_filt\n");

    float t = 0;
    float v_init = 20, s_init = 10;
    float a_real = 5, v_real = v_init, s_real = s_init;
    float a_filt, v_filt, s_filt, s_noisy;


    // purposefully giving it a bad first guess
    KF kf(DT, a_real - 100, v_init + 100, s_init * -2, 0.5);

    for (int i = 0; i < 500; i++) {
        t += DT;
        v_real += DT * a_real;
        s_real += DT * v_real;
        s_noisy = s_real + rand_normal(0.5);

        kf.propagate_dynamics();
        kf.rec_least_squares(s_noisy);
        kf.retrieve_state(s_filt, v_filt, a_filt);
        
        fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, a_real, a_filt, v_real, v_filt, s_real, s_filt);
    }

    fclose(csv);
}