/*
 * kalman.h
 *
 *  Created on: Jul 28, 2024
 *      Author: oguzk
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_



typedef struct {
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float x; // Value
    float P; // Estimation error covariance
    float K; // Kalman gain
} KalmanFilter;

// Initialize Kalman filter
void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_value);

// Update Kalman filter with a new measurement
float KalmanFilter_Update(KalmanFilter *kf, float measurement);


#endif /* INC_KALMAN_H_ */
