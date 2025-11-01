/*
 * EKF.h
 *
 *  Created on: Apr 25, 2025
 *      Author: ASTRAYRMORS
 */

#ifndef EKF_EKF_H_
#define EKF_EKF_H_


#include "arm_math.h"

#define STATE_DIM 7
#define MEAS_DIM 6

// x = [q0 q1 q2 q3 w_x w_y w_z]
typedef struct {
	float q[4];
	float gyro_bias[3];
} EKFState;

// Sensors input
typedef struct {
	float accel[3];
	float gyro[3];
	float mag[3];
} SensorData;

typedef struct {
	float dt;
	arm_matrix_instance_f32 x;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;

	float x_data[STATE_DIM];
	float P_data[STATE_DIM * STATE_DIM];  // 7 * 7
	float Q_data[STATE_DIM * STATE_DIM];  // 7 * 7
	float Qd_buf[STATE_DIM * STATE_DIM];
	float R_data[MEAS_DIM * MEAS_DIM];    // 6 * 6
} EKF;

void EKF_Init(EKF *ekf);
void ekf_update(EKF *ekf, const float accel[3], const float mag[3]);
void ekf_predict(EKF *ekf, const float gyro[3]);
void ekf_predict_2(EKF *ekf, float gyro[3]);
void ekf_get_quaternion(EKF *ekf, float q_out[4]);



#endif /* EKF_EKF_H_ */
