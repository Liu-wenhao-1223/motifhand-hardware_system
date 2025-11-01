/*
 * EKF.c
 *
 *  Created on: Apr 25, 2025
 *      Author: ASTRAYRMORS
 */
#include "EKF.h"

float mx = 0.43, my =-0.41, mz = 0.805;

void EKF_Init(EKF *ekf)
{
//    ekf->dt = dt;                       // Sample Cycle
    /* Matrix Inital */
    arm_mat_init_f32(&ekf->x, STATE_DIM, 1,            ekf->x_data);
    arm_mat_init_f32(&ekf->P, STATE_DIM, STATE_DIM,    ekf->P_data);
    arm_mat_init_f32(&ekf->Q, STATE_DIM, STATE_DIM,    ekf->Q_data);
    arm_mat_init_f32(&ekf->R, MEAS_DIM, MEAS_DIM,      ekf->R_data);

}

static inline void quat_mult(const float p[4],
                             const float q[4],
                             float       out[4])
{
    const float pw = p[0], px = p[1], py = p[2], pz = p[3];
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    out[0] = pw*qw - px*qx - py*qy - pz*qz;              // w
    out[1] = pw*qx + px*qw + py*qz - pz*qy;              // x
    out[2] = pw*qy - px*qz + py*qw + pz*qx;              // y
    out[3] = pw*qz + px*qy - py*qx + pz*qw;              // z
}

void quaternion_to_rotation_matrix(const float q[4], float R[3][3]) {
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    R[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    R[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    R[0][2] = 2.0f * (q1 * q3 + q0 * q2);

    R[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    R[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    R[1][2] = 2.0f * (q2 * q3 - q0 * q1);

    R[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    R[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    R[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
}

/* 7×7 Jacobian: | Fqq  Fqb |
                 | 0    I   |   */
static inline void build_F_matrix(const float q[4],
                                  const float w[3],
                                  float dt,
                                  float *F)      // F 长度 = STATE_DIM*STATE_DIM
{
    /* 1. 单位阵 */
    for (int i = 0; i < STATE_DIM; ++i)
        F[i*STATE_DIM + i] = 1.0f;

    /* 2.  Ω(ω) —— 四元数左乘形式 */
    float wx = w[0], wy = w[1], wz = w[2];

    /* 给 Fqq 加上 0.5·dt·Ω */
    const float halfdt = 0.5f * dt;
#define IDX(r,c)  ((r)*STATE_DIM + (c))
    F[IDX(0,1)] += -halfdt*wx;  F[IDX(0,2)] += -halfdt*wy;  F[IDX(0,3)] += -halfdt*wz;
    F[IDX(1,0)] +=  halfdt*wx;  F[IDX(1,2)] +=  halfdt*wz;  F[IDX(1,3)] += -halfdt*wy;
    F[IDX(2,0)] +=  halfdt*wy;  F[IDX(2,1)] += -halfdt*wz;  F[IDX(2,3)] +=  halfdt*wx;
    F[IDX(3,0)] +=  halfdt*wz;  F[IDX(3,1)] +=  halfdt*wy;  F[IDX(3,2)] += -halfdt*wx;

    /* 3. Fqb = -0.5·dt·Q(q)  —— 四元数对 bias 的偏导 */
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    F[IDX(0,4)] = -halfdt*q1;  F[IDX(0,5)] = -halfdt*q2;  F[IDX(0,6)] = -halfdt*q3;
    F[IDX(1,4)] =  halfdt*q0;  F[IDX(1,5)] =  halfdt*q3;  F[IDX(1,6)] = -halfdt*q2;
    F[IDX(2,4)] = -halfdt*q3;  F[IDX(2,5)] =  halfdt*q0;  F[IDX(2,6)] =  halfdt*q1;
    F[IDX(3,4)] =  halfdt*q2;  F[IDX(3,5)] = -halfdt*q1;  F[IDX(3,6)] =  halfdt*q0;
#undef IDX
}

static void build_H_numeric(const float x[7],
                            void (*h_fun)(const float*, float*),
                            float H[6*7])
{
    const float eps = 1e-4f;
    float x_plus[7], x_minus[7], h_plus[6], h_minus[6];

    memcpy(x_plus,  x, sizeof(float)*7);
    memcpy(x_minus, x, sizeof(float)*7);

    for (int j = 0; j < STATE_DIM; ++j) {
        x_plus [j] += eps;
        x_minus[j] -= eps;

        h_fun(x_plus,  h_plus );
        h_fun(x_minus, h_minus);

        for (int i = 0; i < MEAS_DIM; ++i)
            H[i*STATE_DIM + j] = (h_plus[i] - h_minus[i]) / (2*eps);

        x_plus [j] = x_minus[j] = x[j];     // restore
    }
}

void ekf_predict(EKF *ekf, const float gyro[3]) {
	float *x = ekf->x_data;
	float q[4] = {x[0], x[1], x[2], x[3]};
	float bias[3] = {x[4], x[5], x[6]};
	float w[3] = {
			gyro[0] - bias[0],
			gyro[1] - bias[1],
			gyro[2] - bias[2]
	};


	/* ---------------------------------
	     * 0) 连续 Q_data → 离散 Qd_buf
	     * --------------------------------- */
	const float dt  = ekf->dt;
	const float dt2 = dt * dt;

	/* ① 先把 Qd_buf 清零（只用对角元可省掉 memset）*/
	for (int i = 0; i < STATE_DIM * STATE_DIM; ++i)
		ekf->Qd_buf[i] = 0.0f;

	/* ② 四元数部分：Qc*(½dt)^2   — 连续随机角速率→姿态误差  */
	for (int i = 0; i < 4; ++i) {
		float qc = ekf->Q_data[i*STATE_DIM + i];   // σ_g²  (rad²/s)
		ekf->Qd_buf[i*STATE_DIM + i] = 0.25f * qc * dt2;
	}

	/* ③ 零偏部分：Qc*dt²  — 随机游走 */
	for (int i = 4; i < 7; ++i) {
		float qc = ekf->Q_data[i*STATE_DIM + i];   // σ_bg² (rad²/s²)
		ekf->Qd_buf[i*STATE_DIM + i] = qc * dt2;
	}


	/* ---- ① 角增量用 Δq，不再直接 Euler ----*/
	float half_dt = 0.5f * ekf->dt;
	float ang = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]) * ekf->dt;
	if (ang > 3.14f) ang = 3.14f;                  // 把单步增量限制在 180°
	float sin_half = sinf(ang*0.5f) / (ang*0.5f + 1e-6f);
	float dq[4] = { cosf(ang*0.5f),
	                w[0]*half_dt*sin_half,
	                w[1]*half_dt*sin_half,
	                w[2]*half_dt*sin_half };       // 单位四元数增量

	quat_mult(dq, x, x);                           // q ← dq ⊗ q

	/* ---- ② 归一化前做 NaN / Inf / 0 检查 ----*/
	float nrm2 = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
	if (!isfinite(nrm2) || nrm2 < 1e-6f) {
		x[0]=1.0f; x[1]=x[2]=x[3]=0.0f; nrm2=1.0f;
	}
	float invn = 1.0f / sqrtf(nrm2);
	for (int i = 0; i < 4; ++i) x[i] *= invn;

	float q_now[4] = {x[0], x[1], x[2], x[3]};

//	// Quaternion derivative
//	float dq[4] = {
//		0.5f * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2]),
//		0.5f * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]),
//		0.5f * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]),
//		0.5f * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0])
//	};
//
//	// x = x + dq * dt
//	for (int i = 0; i < 4; i++)
//		x[i] += dq[i] * ekf->dt;
//
//	// Normalize Quaternion
//	float norm = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
//
//	for (int i = 0; i < 4; i++)
//		x[i] /= norm;

//	// P = F * P * F^T + Q （becomes P = P + Q）
//	arm_mat_add_f32(&ekf->P, &ekf->Q, &ekf->P);
	static float F_data[STATE_DIM * STATE_DIM];      
	build_F_matrix(q_now, w, ekf->dt, F_data);         

	arm_matrix_instance_f32 F, FT, FP, FPFt;
	arm_mat_init_f32(&F,  STATE_DIM, STATE_DIM, F_data);

	float FT_data[STATE_DIM * STATE_DIM];
	arm_mat_init_f32(&FT, STATE_DIM, STATE_DIM, FT_data);
	arm_mat_trans_f32(&F, &FT);

	/* ---------- 2. P = F P Fᵀ + Q ---------- */
	float FP_data[STATE_DIM * STATE_DIM];
	float FPFt_data[STATE_DIM * STATE_DIM];

	arm_mat_init_f32(&FP,   STATE_DIM, STATE_DIM, FP_data);
	arm_mat_init_f32(&FPFt, STATE_DIM, STATE_DIM, FPFt_data);

	arm_mat_mult_f32(&F,  &ekf->P, &FP);      // FP  = F·P
	arm_mat_mult_f32(&FP, &FT,      &FPFt);   // FPFᵀ = FP·Fᵀ

	arm_matrix_instance_f32 Qd;
	arm_mat_init_f32(&Qd, STATE_DIM, STATE_DIM, ekf->Qd_buf);

	arm_mat_add_f32 (&FPFt, &Qd, &ekf->P);       // P ← FPFᵀ + Q
//	arm_mat_add_f32 (&FPFt, &ekf->Q, &ekf->P);   // P ← FPFᵀ + Q
}

void ekf_predict_2(EKF *ekf,
				float gyro[3])      // rad/s
{
    /* 1) 拿出当前状态 */
    float *x = ekf->x_data;     // [q0 q1 q2 q3 bgx bgy bgz]
    float dt = ekf->dt;

    /* 2) 去零偏 */
    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];
    float wx = gx - x[4];
    float wy = gy - x[5];
    float wz = gz - x[6];

    /* 3) 四元数微分 */
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float dq0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
    float dq1 = 0.5f * ( q0*wx + q2*wz - q3*wy);
    float dq2 = 0.5f * ( q0*wy - q1*wz + q3*wx);
    float dq3 = 0.5f * ( q0*wz + q1*wy - q2*wx);

    /* 4) 积分 + 归一化 */
    q0 += dq0 * dt;  q1 += dq1 * dt;
    q2 += dq2 * dt;  q3 += dq3 * dt;
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;  q1 /= norm;  q2 /= norm;  q3 /= norm;

    x[0]=q0; x[1]=q1; x[2]=q2; x[3]=q3;
    /* x[4..6] (bias) 保持不变 */

    /* ---------------- 协方差传播 ---------------- */
    /* 5) 构造 F = I + A*dt  (7×7)*/
    float F_data[49] = {0};
    for(int i=0;i<7;i++) F_data[i*7+i]=1.0f; 

    /* -0.5*Omega*dt 填到左上 4×4 */
    F_data[0*7+1] =  0.5f*wx*dt;  F_data[0*7+2] =  0.5f*wy*dt;  F_data[0*7+3] =  0.5f*wz*dt;
    F_data[1*7+0] = -0.5f*wx*dt;  F_data[1*7+2] = -0.5f*wz*dt;  F_data[1*7+3] =  0.5f*wy*dt;
    F_data[2*7+0] = -0.5f*wy*dt;  F_data[2*7+1] =  0.5f*wz*dt;  F_data[2*7+3] = -0.5f*wx*dt;
    F_data[3*7+0] = -0.5f*wz*dt;  F_data[3*7+1] = -0.5f*wy*dt;  F_data[3*7+2] =  0.5f*wx*dt;

    /* -0.5*G(q)*dt  填到左上 4×3 (bias 对 quaternion 的敏感度) */
    F_data[0*7+4] =  0.5f*q1*dt;  F_data[0*7+5] =  0.5f*q2*dt;  F_data[0*7+6] =  0.5f*q3*dt;
    F_data[1*7+4] = -0.5f*q0*dt;  F_data[1*7+5] =  0.5f*q3*dt;  F_data[1*7+6] = -0.5f*q2*dt;
    F_data[2*7+4] = -0.5f*q3*dt;  F_data[2*7+5] = -0.5f*q0*dt;  F_data[2*7+6] =  0.5f*q1*dt;
    F_data[3*7+4] =  0.5f*q2*dt;  F_data[3*7+5] = -0.5f*q1*dt;  F_data[3*7+6] = -0.5f*q0*dt;

    /* 6) 协方差更新：P = F P Fᵀ + L Q Lᵀ          */
    arm_matrix_instance_f32 Fm, Pt, temp;
    arm_mat_init_f32(&Fm,   7,7, F_data);
    arm_mat_init_f32(&Pt,   7,7, ekf->P_data);

    float buf1[49], buf2[49];
    arm_mat_init_f32(&temp, 7,7, buf1);
    arm_mat_mult_f32(&Fm, &Pt, &temp);          // temp = F*P
    arm_mat_init_f32(&Pt, 7,7, buf2);
    arm_mat_trans_f32(&Fm, &Pt);                // Pt = Fᵀ
    arm_mat_mult_f32(&temp, &Pt, &temp);        // temp = F*P*Fᵀ

    /* 过程噪声 LQLᵀ（这里为简单起见，直接近似为 diag(q_rate, b_walk)）
       先把你自己设好的 Qd (7×7) 加到 temp 上 */
    for(int i=0;i<49;i++)
        temp.pData[i] += ekf->Q_data[i];

    /* 把结果写回 P */
    arm_copy_f32(temp.pData, ekf->P_data, 49);
}

float expected_acc[3];
float expected_mag[3];

void compute_expected_measurement(const float x[STATE_DIM], float h[6]) {
	// h[acc[3], mag[3]]
	float q[4] = {x[0], x[1], x[2], x[3]};

	// rotation matrix R(q)
	float R[3][3];
	quaternion_to_rotation_matrix(q, R);

	// Expected acc
	h[0] = -R[2][0];
	h[1] = -R[2][1];
	h[2] = -R[2][2];

	// Expected mag
	h[3] = R[0][0] * mx + R[0][1] * my + R[0][2] * mz;
	h[4] = R[1][0] * mx + R[1][1] * my + R[1][2] * mz;
	h[5] = R[2][0] * mx + R[2][1] * my + R[2][2] * mz;
//	h[3] = R[0][0];
//	h[4] = R[1][0];
//	h[5] = R[2][0];
}

void ekf_update(EKF *ekf, const float accel[3], const float mag[3]) {
	float z[6] = {
		accel[0], accel[1], accel[2],
		mag[0], mag[1], mag[2]
	};

	float q0 = ekf->x_data[0];
	float q1 = ekf->x_data[1];
	float q2 = ekf->x_data[2];
	float q3 = ekf->x_data[3];

	arm_matrix_instance_f32 Z, H, H_T, S, K, Y, I;
	float Z_data[6], H_data[6 * 7], H_T_data[7 * 6], S_data[6 * 6], K_data[7 * 6], Y_data[6], I_data[7 * 7];

	// Construct the matrix H
	build_H_numeric(ekf->x_data, compute_expected_measurement, H_data);

//	memset(H_data, 0, sizeof(H_data));
//
//	/* -------- accel rows -------- */
//	// ∂a_x/∂q
//	H_data[0*STATE_DIM + 0] = -2.0f*q2;
//	H_data[0*STATE_DIM + 1] =  2.0f*q3;
//	H_data[0*STATE_DIM + 2] = -2.0f*q0;
//	H_data[0*STATE_DIM + 3] =  2.0f*q1;
//
//	// ∂a_y/∂q
//	H_data[1*STATE_DIM + 0] =  2.0f*q1;
//	H_data[1*STATE_DIM + 1] =  2.0f*q0;
//	H_data[1*STATE_DIM + 2] =  2.0f*q3;
//	H_data[1*STATE_DIM + 3] =  2.0f*q2;
//
//	// ∂a_z/∂q
//	H_data[2*STATE_DIM + 0] = -2.0f*q0;
//	H_data[2*STATE_DIM + 1] = -2.0f*q1;
//	H_data[2*STATE_DIM + 2] = -2.0f*q2;
//	H_data[2*STATE_DIM + 3] =  2.0f*q3;
//
//	/* -------- mag rows -------- */
//	H_data[3*STATE_DIM + 0] = -2.0f*my*q3 + 2.0f*mz*q2;
//	H_data[3*STATE_DIM + 1] =  2.0f*my*q2 + 2.0f*mz*q3;
//	H_data[3*STATE_DIM + 2] = -4.0f*mx*q2 + 2*my*q1 + 2.0f*mz*q0;
//	H_data[3*STATE_DIM + 3] = -4.0f*mx*q3 + 2*my*q0 + 2.0f*mz*q1;
//
//	H_data[4*STATE_DIM + 0] =  2.0f*mx*q3 - 2.0f*mz*q1;
//	H_data[4*STATE_DIM + 1] =  2.0f*mx*q2 - 4*my*q1 - 2.0f*mz*q0;
//	H_data[4*STATE_DIM + 2] =  2.0f*mx*q1 + 2.0f*mz*q3;
//	H_data[4*STATE_DIM + 3] =  2.0f*mx*q0 - 4*my*q3 + 2.0f*mz*q2;
//
//	H_data[5*STATE_DIM + 0] =  -2.0f*mx*q2 + 2*my*q1;
//	H_data[5*STATE_DIM + 1] =  2.0f*mx*q3 + 2*my*q0 - 4.0f*mz*q1;
//	H_data[5*STATE_DIM + 2] =  -2.0f*mx*q0 + 2*my*q3 - 4.0f*mz*q2;
//	H_data[5*STATE_DIM + 3] =  2.0f*mx*q1 + 2*my*q2;

	//	H_data[3*STATE_DIM + 0] =  0.0f*q0;
//	H_data[3*STATE_DIM + 1] =  0.0f*q1;
//	H_data[3*STATE_DIM + 2] = -4.0f*q2;
//	H_data[3*STATE_DIM + 3] = -4.0f*q3;
//
//	H_data[4*STATE_DIM + 0] =  2.0f*q3;
//	H_data[4*STATE_DIM + 1] =  2.0f*q2;
//	H_data[4*STATE_DIM + 2] =  2.0f*q1;
//	H_data[4*STATE_DIM + 3] =  2.0f*q0;
//
//	H_data[5*STATE_DIM + 0] =  -2.0f*q2;
//	H_data[5*STATE_DIM + 1] =  2.0f*q3;
//	H_data[5*STATE_DIM + 2] =  -2.0f*q0;
//	H_data[5*STATE_DIM + 3] =  2.0f*q1;

	arm_mat_init_f32(&H, MEAS_DIM, STATE_DIM, H_data);
	arm_mat_init_f32(&H_T, STATE_DIM, MEAS_DIM, H_T_data);

	arm_mat_trans_f32(&H, &H_T);

	arm_mat_init_f32(&Z, MEAS_DIM, 1, Z_data);
	memcpy(Z_data, z, sizeof(Z_data));

	// Innovation: y = z - h(x)
	float h_data[6];
	compute_expected_measurement(ekf->x_data, h_data);
	for (int i = 0; i < 6; i++)
		Y_data[i] = Z_data[i] - h_data[i];
	arm_mat_init_f32(&Y, MEAS_DIM, 1, Y_data);

	// S = H*P*H^T + R
	arm_matrix_instance_f32 HP, HPHT;
	float HP_data[6 * 7], HPHT_data[6 * 6];
	arm_mat_init_f32(&HP, MEAS_DIM, STATE_DIM, HP_data);
	arm_mat_init_f32(&HPHT, MEAS_DIM, MEAS_DIM, HPHT_data);
	arm_mat_mult_f32(&H, &ekf->P, &HP);
	arm_mat_mult_f32(&HP, &H_T, &HPHT);

	arm_mat_init_f32(&S, MEAS_DIM, MEAS_DIM, S_data);
	arm_mat_add_f32(&HPHT, &ekf->R, &S);

	// K = P*H^T * S^-1
	arm_matrix_instance_f32 PHT, S_inv;
	float PHT_data[STATE_DIM * MEAS_DIM], S_inv_data[MEAS_DIM * MEAS_DIM];
	arm_mat_init_f32(&PHT, STATE_DIM, MEAS_DIM, PHT_data);
	arm_mat_mult_f32(&ekf->P, &H_T, &PHT);

	arm_mat_init_f32(&S_inv, MEAS_DIM, MEAS_DIM, S_inv_data);
	arm_mat_inverse_f32(&S, &S_inv);

	arm_mat_init_f32(&K, STATE_DIM, MEAS_DIM, K_data);
	arm_mat_mult_f32(&PHT, &S_inv, &K);

	// x = x + K*y
	arm_matrix_instance_f32 X_upd;
	float X_upd_data[STATE_DIM];
	arm_mat_init_f32(&X_upd, STATE_DIM, 1, X_upd_data);
	arm_mat_mult_f32(&K, &Y, &X_upd);
	for (int i = 0; i < STATE_DIM; ++i)
		ekf->x_data[i] += X_upd_data[i];

	// P = (I - K*H)*P
	arm_matrix_instance_f32 KH, I_KH, P_new;
	float KH_data[STATE_DIM * STATE_DIM], I_KH_data[STATE_DIM * STATE_DIM], P_new_data[STATE_DIM * STATE_DIM];

	arm_mat_init_f32(&KH, STATE_DIM, STATE_DIM, KH_data);
	arm_mat_init_f32(&I_KH, STATE_DIM, STATE_DIM, I_KH_data);
	arm_mat_init_f32(&I, STATE_DIM, STATE_DIM, I_data);

	// KH = K * H
	arm_mat_mult_f32(&K, &H, &KH);
	arm_mat_init_f32(&I, STATE_DIM, STATE_DIM, I_data);

	for (int i = 0; i < STATE_DIM * STATE_DIM; ++i)
		I_data[i] = (i % (STATE_DIM + 1)) ? 0.0f : 1.0f;

	// I - KH
	arm_mat_sub_f32(&I, &KH, &I_KH);

	// (I - KH) * P
	arm_mat_init_f32(&P_new, STATE_DIM, STATE_DIM, P_new_data);
	arm_mat_mult_f32(&I_KH, &ekf->P, &P_new);

	memcpy(ekf->P_data, P_new_data, sizeof(P_new_data));

//	float *x = ekf->x_data;
//	float norm = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
//	for (int i = 0; i < 4; i++) x[i] /= norm;

}

void ekf_get_quaternion(EKF *ekf, float q_out[4]) {
    for (int i = 0; i < 4; i++)
        q_out[i] = ekf->x_data[i];
    float q_norm = sqrtf(powf(q_out[0], 2) + powf(q_out[1], 2) + powf(q_out[2], 2) + powf(q_out[3], 2));
    for (int i = 0; i < 4; i++)
    	q_out[i] = q_out[i] / q_norm;
}
