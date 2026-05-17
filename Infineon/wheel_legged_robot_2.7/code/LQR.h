/*
 * LQR.h
 *
 *  Created on: 2025Äê3ÔÂ30ÈÕ
 *      Author: 17104
 */

#ifndef CODE_LQR_H_
#define CODE_LQR_H_

#include "zf_common_headfile.h"
#include "matrix.h"

#define GRAVITY 9.7997
#define WHEEL_RADIUS 0.19       //m
#define WHEEL_DIAMETER 0.0675     //m
#define MAX_ITER 100
#define centroid_distance 60.0f //mm
#define centroid_high_distance (60.0f + 40.0f) //mm
#define EPSILON 1e-6f

extern Matrix A;
extern Matrix B;

float linear_adaptive_q_weight(float x, float x_max, float margin, float max_scale, float K);
float sigmoid_adaptive_q_weight(float x, float K);
float sqrtf_pro(float input);

void LQR_AB_init(Matrix* A, Matrix* B);
void LQR_AB_init_high(Matrix* A, Matrix* B);
void LQR_QR_init(Matrix *Q, float *Q_data, Matrix *R, float *R_data);
int LQR_K_update(Matrix *Q, Matrix *R, Matrix *K);
void get_controller_gains(Matrix* K, float* KL, float* KR, float* Ka);
void get_adaptive_controller_gains(Matrix* K_base, Matrix* K, float* KL, float* KR, float* Ka, float ratio);
void matrix_B_discretizing(Matrix* A, Matrix* B, float Ts, Matrix* res);
//static void householder(float* x, int n, float* v, float* beta);
//static void qr_decomposition(matrix_struct* A, matrix_struct* Q, matrix_struct* R);
//static void qr_algorithm(matrix_struct* A, float* real_eigs, float* imag_eigs);
//static void reorder_schur(matrix_struct* T, matrix_struct* U, int n);
//static void compute_eigenvalues(matrix_struct* A, float* real_eigs, float* imag_eigs);
////int solve_DARE_Schur(matrix_struct *A, matrix_struct *B, matrix_struct *Q, matrix_struct *R, matrix_struct *P);
//void matrix_struct_schur(matrix_struct* A, matrix_struct* T, matrix_struct* U);
//int solve_DARE_Schur(matrix_struct* A, matrix_struct* B, matrix_struct* Q, matrix_struct* R, matrix_struct* P);
//int LQR_K_update_Schur(matrix_struct *Q, matrix_struct *R, matrix_struct *K);

#endif /* CODE_LQR_H_ */
