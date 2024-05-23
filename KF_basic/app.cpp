#include "kf.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define KF_EVERY 5
#define DT 0.1


float rand_normal(float meas_var) {
    float u1 = rand() / (float)RAND_MAX;
    float u2 = rand() / (float)RAND_MAX;
    return sqrt(-2 * log(u1)) * cos(2 * M_PI * u2) * sqrt(meas_var);
}


int main() {
    srand(time(NULL));
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,Accel_real,Accel_noisy,Accel_filt\n");

    KF kf(0.0, 0.0, 0.0, 0.5);
    int step = 0;

    
    // kinematics
    float t = 0;
    float a_real, v_real, s_real;
    float a_noisy;
    float a_filt, v_filt, s_filt;
    a_real = 5.0;



    for (int i = 0; i < 1000; i++) {
        a_real = 5 + sin(t);
        
        t += DT;
        a_noisy = a_real + rand_normal(0.1);

        if (!(step++ % KF_EVERY)) {
            kf.update(a_real, 0.1);
        }
        kf.predict(DT);
        a_filt = kf.x(0,0);
        v_filt = kf.x(1,0);
        s_filt = kf.x(2,0);
        
        fprintf(csv, "%.4f,%.4f,%.4f,%.4f\n", t, a_real, a_noisy, a_filt);
    }



    fclose(csv);
    return 0;
}