/*
 * kalman.c
 *
 *  Created on: Jul 28, 2024
 *      Author: oguzk
 */


#include "kalman.h"

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_value) {
    kf->Q = Q;
    kf->R = R;
    kf->x = initial_value;
    kf->P = 1.0;
    kf->K = 0.0;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // Prediction update
    kf->P += kf->Q;

    // Measurement update
    kf->K = kf->P / (kf->P + kf->R);
    kf->x += kf->K * (measurement - kf->x);
    kf->P *= (1.0 - kf->K);

    return kf->x;
}
