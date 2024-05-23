#include "KF.cpp"
#include "IMU_sim.cpp"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


#define DT 0.05


float drag_coeff() {
    return 0.1;
}


int main() {
    srand(time(NULL));
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,a_real,a_filt,v_real,v_filt,s_real,s_filt\n");

    float t = 0, A = 0.1, cd;
    float a_filt, v_filt, s_filt;


    IMU* imu = (IMU*)malloc(sizeof(IMU));
    imu->a[0] = imu->a[1] = imu->a[2] = 0.0;
    imu->w[0] = imu->w[1] = imu->w[2] = 0.0;
    imu->dt = DT;
    imu->a_real = imu->v_real = imu->s_real = 0.0;


    KF kf(DT, imu->a_real, imu->v_real, imu->s_real, 5, 5, 0.001);

    while (true) {
        t += DT;
        if (imu->s_real < 0 && t > 2) {
            break;
        }

        // HERE THE CONTROL SYSTEM WILL HAVE A BETTER ESTIMATE OF CD TO FEED TO PREDICT;
        cd = drag_coeff();
        imu = generate_model_data(imu, t, A, cd);

        kf.predict(RHO, cd, A, t, MASS);
        kf.update(imu->a[0], imu->s);
        kf.retrieve_state(s_filt, v_filt, a_filt);

        
        fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, imu->a[0], a_filt, imu->v_real, v_filt, imu->s_real, s_filt);
    }

    fclose(csv);
}