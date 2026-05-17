/*
 * LQR.c
 *
 *  Created on: 2025年3月30日
 *      Author: 17104
 */


#include "LQR.h"

Matrix A;
Matrix B;

Matrix reference_P;
int reference_flag = 0;
// x: 当前状态
// x_min, x_max: 状态约束边界
// margin: 安全裕度（如约束的90%开始惩罚）
// max_scale: 最大惩罚倍数（如10.0）
float linear_adaptive_q_weight(float x, float x_max, float margin, float max_scale, float K)
{
    float x_bound = x_max - margin; // 实际开始惩罚的边界
    float normalized_dist = fabsf(x) / x_bound; // 归一化距离

    if (normalized_dist <= 1.0f) {
        return K; // 安全区域内不惩罚
    } else {
        // 线性惩罚：越界越远，惩罚越大
        return K * sqrtf(1.0f + (max_scale - 1.0f) * (normalized_dist - 1.0f));
    }

}

float sigmoid_adaptive_q_weight(float x, float K)
{
    return K * sqrtf(1 + 8 / (1 + expf(-0.5 * (fabsf(x) - 10))));
}

float sqrtf_pro(float input)
{
    return input >= 0 ? sqrt(input) : sqrt(-input);
}

void LQR_AB_init(Matrix* A, Matrix* B) {
    float A23, A43, B21, B22, B23, A24, B41, B42, B43, B61, B62, B74, B75, Qeq;
    float g = GRAVITY;                     //重力加速度m/s^2
    float m = 37.7f / 1000.0f;         //车轮质量kg
    float r = WHEEL_DIAMETER / 2.0f;  //车轮半径
    float I = 0.000265f;               //车轮转动惯量kg*m^2
    float d = WHEEL_RADIUS;        //轮距
    float M = 1247.0f / 1000.0f;       //车体质量kg
    float l = centroid_distance / 1000.0f;           //质心距离底盘中心的距离m
    float width = 100.0f / 1000.0f;     //车体宽度m
    float length = 170.0f / 1000.0f;   //车体长度m
    float height = 90.0f / 1000.0f;    //车体高度m
    float Jp = (M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l;  //车体绕质心俯仰旋转的转动惯量kg*m^2
    float Jc = (M - 2 * m) * (length * length + width * width) / 12.0f;    //车体绕质心偏航旋转的转动惯量kg*m^2

    Qeq = Jp * M + (Jp + M * l * l) * (2 * m + 2 * I / r / r);
    A23 = -1 * M * M * l * l * g / Qeq;
    B23 = A23;
    A24 = -1 * 0.3f * M * l / (M + 2 * m + 2 * I / r / r);  //俯仰-平动耦合
    A43 = M * l * g * (M + 2 * m + 2 * I / r / r) / Qeq;
    B43 = A43;
    B21 = (Jp + M * l * l + M * l * r) / Qeq / r;
    B22 = (Jp + M * l * l + M * l * r) / Qeq / r;
    B41 = -1 * (M * l / r + M + 2 * m + 2 * I / r / r) / Qeq;
    B42 = -1 * (M * l / r + M + 2 * m + 2 * I / r / r) / Qeq;
    B61 = 1 / r / (m * d + I * d / r / r + 2 * Jc / d);
    B62 = -1 / r / (m * d + I * d / r / r + 2 * Jc / d);
    B74 = -1.0f / d;
    B75 = 1.0f / d;

    float A_data[7][7] = {
        { 0, -1, 0, 0, 0, 0, 0 },
        { 0, 0, -1 * A23, A24, 0, 0, 0 },
        { 0, 0, 0, 1, 0, 0, 0 },
        { 0, 0, A43, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, -1, 0 },
        { 0, 0, 0, 0, 0, 0, 0},
        { 0, 0, 0, 0, 0, 0, 0} };
    matrix_init(7, 7, A, &A_data[0][0]);

    float B_data[7][5] = {
        { 0, 0, 0, 0, 0 },
        { -1 * B21, -1 * B22, -1 * B23, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { B41, B42, B43, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { B61, B62, 0, 0, 0 },
        { 0, 0, 0, B74, B75 } };
    matrix_init(7, 5, B, &B_data[0][0]);
}

void LQR_AB_init_high(Matrix* A, Matrix* B)
{
    float A23, A24, A43, B21, B22, B23, B41, B42, B43, B61, B62, B74, B75, Qeq;
    float g = GRAVITY;                     //重力加速度m/s^2
    float m = 37.7f / 1000.0f;         //车轮质量kg
    float r = WHEEL_DIAMETER / 2.0f;  //车轮半径
    float I = 0.000265f;               //车轮转动惯量kg*m^2
    float d = WHEEL_RADIUS;        //轮距
    float M = 1247.0f / 1000.0f;       //车体质量kg
    float l = centroid_high_distance / 1000.0f;           //质心距离底盘中心的距离m
    float width = 100.0f / 1000.0f;     //车体宽度m
    float length = 170.0f / 1000.0f;   //车体长度m
    float height = 90.0f / 1000.0f;    //车体高度m
    float Jp = (M - 2 * m) * (length * length + height * height) / 12.0f + (M - 2 * m) * l * l;  //车体绕质心俯仰旋转的转动惯量kg*m^2
    float Jc = (M - 2 * m) * (length * length + width * width) / 12.0f;    //车体绕质心偏航旋转的转动惯量kg*m^2

    Qeq = Jp * M + (Jp + M * l * l) * (2 * m + 2 * I / r / r);
    A23 = -1 * M * M * l * l * g / Qeq;
    B23 = A23;
    A24 = -1 * 0.3f * M * l / (M + 2 * m + 2 * I / r / r);  //俯仰-平动耦合
    A43 = M * l * g * (M + 2 * m + 2 * I / r / r) / Qeq;
    B43 = A43;
    B21 = (Jp + M * l * l + M * l * r) / Qeq / r;
    B22 = (Jp + M * l * l + M * l * r) / Qeq / r;
    B41 = -1 * (M * l / r + M + 2 * m + 2 * I / r / r) / Qeq;
    B42 = -1 * (M * l / r + M + 2 * m + 2 * I / r / r) / Qeq;
    B61 = 1 / r / (m * d + I * d / r / r + 2 * Jc / d);
    B62 = -1 / r / (m * d + I * d / r / r + 2 * Jc / d);
    B74 = -1.0f / d;
    B75 = 1.0f / d;

    float A_data[7][7] = {
        { 0, -1, 0, 0, 0, 0, 0 },
        { 0, 0, -1 * A23, A24, 0, 0, 0 },
        { 0, 0, 0, 1, 0, 0, 0 },
        { 0, 0, A43, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, -1, 0 },
        { 0, 0, 0, 0, 0, 0, 0},
        { 0, 0, 0, 0, 0, 0, 0} };
    matrix_init(7, 7, A, &A_data[0][0]);

    float B_data[7][5] = {
        { 0, 0, 0, 0, 0 },
        { -1 * B21, -1 * B22, -1 * B23, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { B41, B42, B43, 0, 0 },
        { 0, 0, 0, 0, 0 },
        { B61, B62, 0, 0, 0 },
        { 0, 0, 0, B74, B75 } };
    matrix_init(7, 5, B, &B_data[0][0]);
}

void LQR_QR_init(Matrix *Q, float *Q_data, Matrix *R, float *R_data) {
    matrix_diag_init(7, Q, Q_data);
    matrix_diag_init(5, R, R_data);
}

int LQR_K_update(Matrix *Q, Matrix *R, Matrix *K) {
    // 初始化所有矩阵
    Matrix P, prev_P, Ad, Bd, Atra, Btra, Rinv;
    Matrix Atra_P, Atra_P_A, Atra_P_B, Btra_P, Btra_P_B, RaddBtra_P_B;
    Matrix RaddBtra_P_B_inv, Btra_P_A, Atra_P_B_RaddBtra_P_B_inv, Atra_P_B_RaddBtra_P_B_inv_Btra_P_A, Atra_P_AsubAtra_P_B_RaddBtra_P_B_inv_Btra_P_A, delta_P;

    matrix_zero_init(7, 7, &P);
    matrix_zero_init(7, 7, &prev_P);

    if(!reference_flag)
    {
        matrix_copy(Q, &P);
        matrix_copy(Q, &prev_P);
        matrix_zero_init(7, 7, &reference_P);
    }
    else
    {
        matrix_copy(&reference_P, &P);
        matrix_copy(&reference_P, &prev_P);
    }

//    uint32 Ts = system_getval_ms();
    //A、B矩阵的零阶保持离散化
    matrix_zero_init(7, 7, &Ad);
    matrix_exp(&A, 0.005f, &Ad);

    matrix_zero_init(7, 5, &Bd);
    matrix_B_discretizing(&A, &B, 0.005f, &Bd);
//    printf("time: %lu ms\n", system_getval_ms() - Ts);
//    Matrix A, B;  // 全局变量改为局部变量
//    LQR_AB_init(&A, &B);  // 重新初始化A,B

    matrix_zero_init(7, 7, &Atra);
    matrix_transpose(&Ad, &Atra);

    matrix_zero_init(5, 7, &Btra);  // 修正维度
    matrix_transpose(&Bd, &Btra);

    matrix_zero_init(5, 5, &Rinv);  // 修正维度
    matrix_inverse(R, &Rinv);

    // 迭代求解
    int count = 0;
    float err = 0.0f;
    const float tolerance = 0.001f;
    const int max_iterations = 1000;
//    uint32 Ts = system_getval_ms();
    while (count < max_iterations) {
        // Atra_P = A' * P
        matrix_mul(&Atra, &prev_P, &Atra_P);

        // Atra_P_A = A' * P * A
        matrix_mul(&Atra_P, &Ad, &Atra_P_A);

        // Atra_P_B = A' * P * B
        matrix_mul(&Atra_P, &Bd, &Atra_P_B);

        // Btra_P = B' * P
        matrix_mul(&Btra, &prev_P, &Btra_P);

        // Btra_P_B = B' * P * B
        matrix_mul(&Btra_P, &Bd, &Btra_P_B);

        // Btra_P_A = B' * P * A
        matrix_mul(&Btra_P, &Ad, &Btra_P_A);

        // R + B' * P * B
        matrix_add(R, &Btra_P_B, &RaddBtra_P_B);

        // (R + B' * P * B)^-1
        matrix_inverse(&RaddBtra_P_B, &RaddBtra_P_B_inv);

        // temp1 = A' * P * B * (R + B' * P * B)^-1
        matrix_mul(&Atra_P_B, &RaddBtra_P_B_inv, &Atra_P_B_RaddBtra_P_B_inv);

        // temp2 = temp1 * B' * P * A
        matrix_mul(&Atra_P_B_RaddBtra_P_B_inv, &Btra_P_A, &Atra_P_B_RaddBtra_P_B_inv_Btra_P_A);

        // P = A' * P * A - temp2 + Q
        matrix_sub(&Atra_P_A, &Atra_P_B_RaddBtra_P_B_inv_Btra_P_A, &Atra_P_AsubAtra_P_B_RaddBtra_P_B_inv_Btra_P_A);

        matrix_add(&Atra_P_AsubAtra_P_B_RaddBtra_P_B_inv_Btra_P_A, Q, &P);

        // 检查收敛性
        matrix_sub(&P, &prev_P, &delta_P);
        err = matrix_frobenius_norm(&delta_P);
//        printf("count: %d err: %.8f\n",count,err);
        count++;

        if (err < tolerance) {
            break;
        }

        // 更新P矩阵
        matrix_copy(&P, &prev_P);
    }

    // 计算最终K矩阵: K = (R + B' * P * B)^-1 * B' * P * A
    matrix_zero_init(5, 7, K);  // 修正维度

    // 重新计算B' * P (使用最新的P)
    matrix_mul(&Btra, &P, &Btra_P);

    // B' * P * B
    matrix_mul(&Btra_P, &Bd, &Btra_P_B);

    // R + B' * P * B
    matrix_add(R, &Btra_P_B, &RaddBtra_P_B);

    // (R + B' * P * B)^-1
    matrix_inverse(&RaddBtra_P_B, &RaddBtra_P_B_inv);

    // B' * P * A
    matrix_mul(&Btra_P, &Ad, &Btra_P_A);

    // K = (R + B' * P * B)^-1 * B' * P * A
    matrix_mul(&RaddBtra_P_B_inv, &Btra_P_A, K);
//    printf("time: %lu ms\n", system_getval_ms() - Ts);
    if(!reference_flag)
    {
        reference_flag = 1;
        matrix_copy(&P, &reference_P);
    }

//    matrix_show(K);
//    matrix_show(&reference_P);
    // 释放所有矩阵
    matrix_free(&P);
    matrix_free(&prev_P);
    matrix_free(&Ad);
    matrix_free(&Bd);
    matrix_free(&Atra);
    matrix_free(&Btra);
    matrix_free(&Rinv);
    matrix_free(&Atra_P);
    matrix_free(&Atra_P_A);
    matrix_free(&Atra_P_B);
    matrix_free(&Btra_P);
    matrix_free(&Btra_P_B);
    matrix_free(&RaddBtra_P_B);
    matrix_free(&RaddBtra_P_B_inv);
    matrix_free(&Btra_P_A);
    matrix_free(&Atra_P_B_RaddBtra_P_B_inv);
    matrix_free(&Atra_P_B_RaddBtra_P_B_inv_Btra_P_A);
    matrix_free(&Atra_P_AsubAtra_P_B_RaddBtra_P_B_inv_Btra_P_A);
    matrix_free(&delta_P);
//    matrix_free(&A);
//    matrix_free(&B);

    return count;
}

void get_controller_gains(Matrix* K, float* KL, float* KR, float* Ka) {
    if (!matrix_is_valid(K) || !KL || !KR || !Ka) return;

    const float* data = matrix_const_data_ptr(K);
    const uint8_t cols = K->cols;

    // 确保矩阵有至少3行
    if (K->rows < 3) return;

    for (uint8_t i = 0; i < cols; i++) {
        KL[i] = data[0 * cols + i];  // 第0行
        KR[i] = data[1 * cols + i];  // 第1行
        Ka[i] = data[2 * cols + i];  // 第2行
//        printf("%.10f,%.10f,%.10f\n", KL[i], KR[i], Ka[i]);
    }
}

void get_adaptive_controller_gains(Matrix* K_base, Matrix* K, float* KL, float* KR, float* Ka, float ratio) {
    if (!matrix_is_valid(K_base) ||!matrix_is_valid(K) || !KL || !KR || !Ka) return;

    const float* data_base = matrix_const_data_ptr(K_base);
    const float* data = matrix_const_data_ptr(K);
    const uint8_t cols = K->cols;
    if(ratio > 1){ratio = 1;}
    if(ratio < 0){ratio = 0;}
//    ratio = ratio > 1 ? 1 : (ratio < 0 ? 0 : ratio);
    // 确保矩阵有至少3行
    if (K_base->rows < 3) return;
    if (K->rows < 3) return;

    for (uint8_t i = 0; i < cols; i++) {
        KL[i] = data_base[0 * cols + i] * (1 - ratio) + data[0 * cols + i] * ratio;  // 第0行
        KR[i] = data_base[1 * cols + i] * (1 - ratio) + data[1 * cols + i] * ratio;  // 第1行
        Ka[i] = data_base[2 * cols + i] * (1 - ratio) + data[2 * cols + i] * ratio;  // 第2行
//        printf("%.10f,%.10f,%.10f\n", KL[i], KR[i], Ka[i]);
    }
}

void matrix_B_discretizing(Matrix* A, Matrix* B, float Ts, Matrix* res) {
    if (!matrix_is_square(A) || A->rows != B->rows) return;

    Matrix temp0, temp1, temp3;
    matrix_zero_init(A->rows, A->cols, &temp1);

    for (int i = 0; i < 20; i++) {
        matrix_iden_init(A->cols, &temp0);

        for (int j = 0; j < i; j++) {
            matrix_mul(&temp0, A, &temp3);
            matrix_copy(&temp3, &temp0);
        }

        float temp2 = powf(Ts, (float)(i + 1)) / (float)factorial(i + 1);
        matrix_scalar_mul(&temp0, temp2, &temp0);

        matrix_add(&temp1, &temp0, &temp1);
    }

    matrix_mul(&temp1, B, res);

    // 释放临时矩阵
    matrix_free(&temp0);
    matrix_free(&temp1);
    matrix_free(&temp3);
}

//static void householder(float* x, int n, float* v, float* beta) {
//    float sigma = 0.0f;
//    float x1 = x[0];
//    float mu = 0.0f;
//
//    // 计算x的2范数
//    for (int i = 1; i < n; i++) {
//        sigma += x[i] * x[i];
//    }
//
//    if (sigma < EPSILON) {
//        *beta = 0.0f;
//        v[0] = 0.0f;
//    } else {
//        mu = sqrtf(x1 * x1 + sigma);
//        if (x1 <= 0) {
//            v[0] = x1 - mu;
//        } else {
//            v[0] = -sigma / (x1 + mu);
//        }
//
//        *beta = 2 * v[0] * v[0] / (sigma + v[0] * v[0]);
//        for (int i = 1; i < n; i++) {
//            v[i] = x[i] / v[0];
//        }
//        v[0] = 1.0f;
//    }
//}
//
//static void qr_decomposition(matrix_struct* A, matrix_struct* Q, matrix_struct* R) {
//    int n = A->row;
//    float v[n], x[n];
//    float beta;
//
//    matrix_struct_copy(A, R);
//    matrix_struct_zero_init(n, n, Q);
//    for (int i = 0; i < n; i++) {
//        Q->data[i][i] = 1.0f;
//    }
//
//    for (int k = 0; k < n-1; k++) {
//        // 提取第k列的下部分
//        for (int i = k; i < n; i++) {
//            x[i-k] = R->data[i][k];
//        }
//
//        // 计算Householder变换
//        householder(x, n-k, v, &beta);
//
//        // 应用变换到R
//        for (int j = k; j < n; j++) {
//            float dot = 0.0f;
//            for (int i = k; i < n; i++) {
//                dot += v[i-k] * R->data[i][j];
//            }
//            for (int i = k; i < n; i++) {
//                R->data[i][j] -= beta * v[i-k] * dot;
//            }
//        }
//
//        // 累积变换到Q
//        for (int i = 0; i < n; i++) {
//            float dot = 0.0f;
//            for (int j = k; j < n; j++) {
//                dot += Q->data[i][j] * v[j-k];
//            }
//            for (int j = k; j < n; j++) {
//                Q->data[i][j] -= beta * dot * v[j-k];
//            }
//        }
//    }
//}
//
//static void qr_algorithm(matrix_struct* A, float* real_eigs, float* imag_eigs) {
//    int n = A->row;
//    matrix_struct Ak, Q, R;
//    matrix_struct_zero_init(n, n, &Ak);
//    matrix_struct_zero_init(n, n, &Q);
//    matrix_struct_zero_init(n, n, &R);
//
//    matrix_struct_copy(A, &Ak);
//
//    for (int iter = 0; iter < MAX_ITER; iter++) {
//        // 使用位移加速收敛
//        float mu = Ak.data[n-1][n-1];
//        for (int i = 0; i < n; i++) {
//            Ak.data[i][i] -= mu;
//        }
//
//        qr_decomposition(&Ak, &Q, &R);
//
//        // Ak = R*Q + mu*I
//        matrix_struct_mul(&R, &Q, &Ak);
//        for (int i = 0; i < n; i++) {
//            Ak.data[i][i] += mu;
//        }
//
//        // 检查次对角线元素是否足够小
//        int converged = 1;
//        for (int i = 1; i < n; i++) {
//            if (fabsf(Ak.data[i][i-1]) > EPSILON) {
//                converged = 0;
//                break;
//            }
//        }
//        if (converged) break;
//    }
//
//    // 提取特征值
//    for (int i = 0; i < n; ) {
//        if (i == n-1 || fabsf(Ak.data[i+1][i]) < EPSILON) {
//            // 实特征值
//            real_eigs[i] = Ak.data[i][i];
//            imag_eigs[i] = 0.0f;
//            i++;
//        } else {
//            // 复共轭对
//            float a = Ak.data[i][i];
//            float b = Ak.data[i][i+1];
//            float c = Ak.data[i+1][i];
//            float d = Ak.data[i+1][i+1];
//
//            float trace = a + d;
//            float det = a * d - b * c;
//            float disc = trace * trace - 4 * det;
//
//            if (disc >= 0) {
//                // 实际上应该是复数，但数值误差导致判别式为正
//                real_eigs[i] = (trace + sqrtf(disc)) / 2;
//                real_eigs[i+1] = (trace - sqrtf(disc)) / 2;
//                imag_eigs[i] = imag_eigs[i+1] = 0.0f;
//            } else {
//                real_eigs[i] = real_eigs[i+1] = trace / 2;
//                imag_eigs[i] = sqrtf(-disc) / 2;
//                imag_eigs[i+1] = -imag_eigs[i];
//            }
//            i += 2;
//        }
//    }
//}
//
//static void compute_eigenvalues(matrix_struct* A, float* real_eigs, float* imag_eigs) {
//    if (A->row == 2) {
//        // 2x2矩阵直接计算
//        float a = A->data[0][0], b = A->data[0][1];
//        float c = A->data[1][0], d = A->data[1][1];
//
//        float trace = a + d;
//        float det = a * d - b * c;
//        float disc = trace * trace - 4 * det;
//
//        if (disc >= 0) {
//            real_eigs[0] = (trace + sqrtf(disc)) / 2;
//            real_eigs[1] = (trace - sqrtf(disc)) / 2;
//            imag_eigs[0] = imag_eigs[1] = 0;
//        } else {
//            real_eigs[0] = real_eigs[1] = trace / 2;
//            imag_eigs[0] = sqrtf(-disc) / 2;
//            imag_eigs[1] = -imag_eigs[0];
//        }
//    } else {
//        // 对于7x7矩阵使用QR算法
//        qr_algorithm(A, real_eigs, imag_eigs);
//
//        // 可选：对特征值进行排序
//        for (int i = 0; i < A->row; i++) {
//            for (int j = i+1; j < A->row; j++) {
//                // 按实部降序排列
//                if (real_eigs[i] < real_eigs[j] ||
//                    (real_eigs[i] == real_eigs[j] && fabs(imag_eigs[i]) < fabs(imag_eigs[j]))) {
//                    float temp_real = real_eigs[i];
//                    float temp_imag = imag_eigs[i];
//                    real_eigs[i] = real_eigs[j];
//                    imag_eigs[i] = imag_eigs[j];
//                    real_eigs[j] = temp_real;
//                    imag_eigs[j] = temp_imag;
//                }
//            }
//        }
//    }
//}
//
//// Schur分解主函数
//void matrix_struct_schur(matrix_struct* A, matrix_struct* T, matrix_struct* U) {
//    int n = A->row;
//    float eps = 1e-6f;
//    int max_iter = 100;
//
//    // 使用动态分配避免堆栈溢出
//    matrix_struct* columns = (matrix_struct*)malloc(n * sizeof(matrix_struct));
//    if (!columns) {
//        // 错误处理
//        fprintf(stderr, "[ERROR] Schur decomposition failed: Cannot allocate %d bytes\n",
//                   n * (int)sizeof(matrix_struct));
//        return;
//    }
//
//    // 初始化U为单位矩阵
//    matrix_struct_zero_init(n, n, U);
//    for (int i = 0; i < n; i++) {
//        U->data[i][i] = 1.0f;
//    }
//
//    // 复制A到T
//    matrix_struct_copy(A, T);
//
//    for (int iter = 0; iter < max_iter; iter++) {
//        // 检查是否已经是上三角矩阵
//        int is_upper = 1;
//        for (int i = 1; i < n; i++) {
//            for (int j = 0; j < i; j++) {
//                if (fabsf(T->data[i][j]) > eps) {
//                    is_upper = 0;
//                    break;
//                }
//            }
//            if (!is_upper) break;
//        }
//        if (is_upper) break;
//
//        // 计算位移
//        float mu = T->data[n-1][n-1];
//
//        // T - mu*I
//        matrix_struct T_minus_muI, Q, R;
//        matrix_struct_zero_init(n, n, &T_minus_muI);
//        matrix_struct_zero_init(n, n, &Q);
//        matrix_struct_zero_init(n, n, &R);
//
//        matrix_struct_copy(T, &T_minus_muI);
//        for (int i = 0; i < n; i++) {
//            T_minus_muI.data[i][i] -= mu;
//        }
//
//        // QR分解 - 修正后的实现
//        for (int j = 0; j < n; j++) {
//            // 初始化列向量
//            matrix_struct_zero_init(n, 1, &columns[j]);
//            for (int i = 0; i < n; i++) {
//                columns[j].data[i][0] = T_minus_muI.data[i][j];
//            }
//
//            // Gram-Schmidt正交化
//            for (int k = 0; k < j; k++) {
//                float dot = 0.0f;
//                for (int i = 0; i < n; i++) {
//                    dot += Q.data[i][k] * columns[j].data[i][0];
//                }
//
//                for (int i = 0; i < n; i++) {
//                    columns[j].data[i][0] -= dot * Q.data[i][k];
//                }
//            }
//
//            // 归一化
//            float norm = 0.0f;
//            for (int i = 0; i < n; i++) {
//                norm += columns[j].data[i][0] * columns[j].data[i][0];
//            }
//            norm = sqrtf(norm);
//
//            if (norm > eps) {
//                for (int i = 0; i < n; i++) {
//                    Q.data[i][j] = columns[j].data[i][0] / norm;
//                }
//            }
//
//            // 填充R矩阵
//            for (int k = 0; k <= j; k++) {
//                R.data[k][j] = 0.0f;
//                for (int i = 0; i < n; i++) {
//                    R.data[k][j] += Q.data[i][k] * T_minus_muI.data[i][j];
//                }
//            }
//        }
//
//        // 更新T = R*Q + mu*I
//        matrix_struct_mul(&R, &Q, T);
//        for (int i = 0; i < n; i++) {
//            T->data[i][i] += mu;
//        }
//
//        // 更新U = U*Q
//        matrix_struct tempU;
//        matrix_struct_zero_init(n, n, &tempU);
//        matrix_struct_mul(U, &Q, &tempU);
//        matrix_struct_copy(&tempU, U);
//
//        // 释放临时矩阵
//        for (int j = 0; j < n; j++) {
//            // 释放columns[j]的内存
//            // (根据你的矩阵库实现可能需要特定释放函数)
//        }
//    }
//
//    free(columns);
//}
//
//// 重新排序Schur分解以分离稳定和不稳定子空间
//static void reorder_schur(matrix_struct* T, matrix_struct* U, int n) {
//    float eps = 1e-6f;
//    int* selected = (int*)malloc(n * sizeof(int));
//
//    // 首先标记所有特征值是否在单位圆内（稳定）
//    float real_eigs[n], imag_eigs[n];
//    compute_eigenvalues(T, real_eigs, imag_eigs);
//
//    for (int i = 0; i < n; i++) {
//        float mag = sqrtf(real_eigs[i]*real_eigs[i] + imag_eigs[i]*imag_eigs[i]);
//        selected[i] = (mag < 1.0f - eps) ? 1 : 0;
//    }
//
//    // 重新排序使稳定的特征值在前
//    for (int i = 0; i < n; i++) {
//        if (!selected[i]) {
//            for (int j = i+1; j < n; j++) {
//                if (selected[j]) {
//                    // 交换i和j的位置
//                    // 交换T的行和列
//                    for (int k = 0; k < n; k++) {
//                        float temp = T->data[k][i];
//                        T->data[k][i] = T->data[k][j];
//                        T->data[k][j] = temp;
//                    }
//                    for (int k = 0; k < n; k++) {
//                        float temp = T->data[i][k];
//                        T->data[i][k] = T->data[j][k];
//                        T->data[j][k] = temp;
//                    }
//                    // 交换U的列
//                    for (int k = 0; k < n; k++) {
//                        float temp = U->data[k][i];
//                        U->data[k][i] = U->data[k][j];
//                        U->data[k][j] = temp;
//                    }
//                    // 交换标记
//                    int temp_sel = selected[i];
//                    selected[i] = selected[j];
//                    selected[j] = temp_sel;
//                    break;
//                }
//            }
//        }
//    }
//
//    free(selected);
//}
//
//// 解离散代数Riccati方程(DARE)的Schur方法
//int solve_DARE_Schur(matrix_struct* A, matrix_struct* B, matrix_struct* Q, matrix_struct* R, matrix_struct* P) {
//    int n = A->row;
//    int m = B->column;
//
//    // 构造Hamiltonian矩阵
//    matrix_struct H, At, Bt, Rinv, temp1, temp2, At_inv;
//    matrix_struct_zero_init(2*n, 2*n, &H);
//    matrix_struct_zero_init(n, n, &At);
//    matrix_struct_zero_init(m, n, &Bt);
//    matrix_struct_zero_init(m, m, &Rinv);
//    matrix_struct_zero_init(n, n, &temp1);
//    matrix_struct_zero_init(n, n, &temp2);
//    matrix_struct_zero_init(n, n, &At_inv);
//
//    // 计算转置和逆
//    matrix_struct_tran(A, &At);
//    matrix_struct_tran(B, &Bt);
//    matrix_struct_inv(R, &Rinv);
//    matrix_struct_inv(&At, &At_inv);
//
//    // 计算B*R^-1*B'
//    matrix_struct BRiBt;
//    matrix_struct_zero_init(n, n, &BRiBt);
//    matrix_struct_mul(B, &Rinv, &temp1);
//    matrix_struct_mul(&temp1, &Bt, &BRiBt);
//
//    // 计算A'^-1*Q
//    matrix_struct AtiQ;
//    matrix_struct_zero_init(n, n, &AtiQ);
//    matrix_struct_mul(&At_inv, Q, &AtiQ);
//
//    // 填充Hamiltonian矩阵
//    // H = [A - B*R^-1*B'*A'^-1*Q   -B*R^-1*B'*A'^-1;
//    //      -A'^-1*Q                 A'^-1]
//
//    // 左上块: A - B*R^-1*B'*A'^-1*Q
//    matrix_struct_mul(&BRiBt, &AtiQ, &temp1);
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            H.data[i][j] = A->data[i][j] - temp1.data[i][j];
//        }
//    }
//
//    // 右上块: -B*R^-1*B'*A'^-1
//    matrix_struct_mul(&BRiBt, &At_inv, &temp1);
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            H.data[i][n+j] = -temp1.data[i][j];
//        }
//    }
//
//    // 左下块: -A'^-1*Q
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            H.data[n+i][j] = -AtiQ.data[i][j];
//        }
//    }
//
//    // 右下块: A'^-1
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            H.data[n+i][n+j] = At_inv.data[i][j];
//        }
//    }
//
//    // 计算H的Schur分解
//    matrix_struct T, U;
//    matrix_struct_zero_init(2*n, 2*n, &T);
//    matrix_struct_zero_init(2*n, 2*n, &U);
//    matrix_struct_schur(&H, &T, &U);
//
//    // 重新排序Schur分解
//    reorder_schur(&T, &U, 2*n);
//
//    // 提取U11和U21
//    matrix_struct U11, U21;
//    matrix_struct_zero_init(n, n, &U11);
//    matrix_struct_zero_init(n, n, &U21);
//
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            U11.data[i][j] = U.data[i][j];
//            U21.data[i][j] = U.data[n+i][j];
//        }
//    }
//
//    // 计算P = U21 * U11^-1
//    matrix_struct U11_inv;
//    matrix_struct_zero_init(n, n, &U11_inv);
//    matrix_struct_inv(&U11, &U11_inv);
//    matrix_struct_mul(&U21, &U11_inv, P);
//
//    // 确保P对称
//    for (int i = 0; i < n; i++) {
//        for (int j = i+1; j < n; j++) {
//            P->data[i][j] = P->data[j][i] = 0.5f * (P->data[i][j] + P->data[j][i]);
//        }
//    }
//
//    return 0;
//}
//
////int solve_DARE_Schur(matrix_struct* A, matrix_struct* B, matrix_struct* Q, matrix_struct* R, matrix_struct* P) {
////    int n = A->rows;
////    int m = B->cols;
////
////    // 阶段1：预计算所有中间矩阵（按使用顺序）
////    matrix_struct At, Bt, Rinv, At_inv;
////    matrix_struct_zero_init(n, n, &At);
////    matrix_struct_zero_init(m, n, &Bt);
////    matrix_struct_zero_init(m, m, &Rinv);
////    matrix_struct_zero_init(n, n, &At_inv);
////
////    matrix_struct_tran(A, &At);
////    matrix_struct_tran(B, &Bt);
////    matrix_struct_inv(R, &Rinv);
////    matrix_struct_inv(&At, &At_inv);
////
////    // 阶段2：计算BRiBt和AtiQ后立即释放不需要的矩阵
////    matrix_struct BRiBt, AtiQ;
////    {
////        matrix_struct temp1;
////        matrix_struct_zero_init(n, m, &temp1);
////        matrix_struct_mul(B, &Rinv, &temp1);
////        matrix_struct_zero_init(n, n, &BRiBt);
////        matrix_struct_mul(&temp1, &Bt, &BRiBt);
////        // 立即释放temp1
////        matrix_struct_free(&temp1); // 假设有释放函数
////    }
////
////    {
////        matrix_struct_zero_init(n, n, &AtiQ);
////        matrix_struct_mul(&At_inv, Q, &AtiQ);
////        // 立即释放At_inv (不再需要)
////        matrix_struct_free(&At_inv);
////    }
////
////    // 阶段3：构建Hamiltonian矩阵（分块填充）
////    matrix_struct H;
////    matrix_struct_zero_init(2*n, 2*n, &H);
////
////    // 填充H矩阵左上块: A - BRiBt*AtiQ
////    {
////        matrix_struct temp;
////        matrix_struct_zero_init(n, n, &temp);
////        matrix_struct_mul(&BRiBt, &AtiQ, &temp);
////        for (int i = 0; i < n; i++) {
////            for (int j = 0; j < n; j++) {
////                H.data[i][j] = A->data[i][j] - temp.data[i][j];
////            }
////        }
////        matrix_struct_free(&temp);
////        matrix_struct_free(&BRiBt); // 不再需要
////    }
////
////    // 填充H矩阵右上块: -BRiBt*At_inv (需要重新计算)
////    {
////        matrix_struct temp1, temp2;
////        matrix_struct_zero_init(n, m, &temp1);
////        matrix_struct_mul(B, &Rinv, &temp1);
////        matrix_struct_zero_init(n, n, &temp2);
////        matrix_struct_mul(&temp1, &Bt, &temp2);
////
////        for (int i = 0; i < n; i++) {
////            for (int j = 0; j < n; j++) {
////                H.data[i][n+j] = -temp2.data[i][j];
////            }
////        }
////        matrix_struct_free(&temp1);
////        matrix_struct_free(&temp2);
////        matrix_struct_free(&Rinv); // 不再需要
////    }
////
////    // 填充H矩阵左下块: -AtiQ
////    for (int i = 0; i < n; i++) {
////        for (int j = 0; j < n; j++) {
////            H.data[n+i][j] = -AtiQ.data[i][j];
////        }
////    }
////    matrix_struct_free(&AtiQ); // 不再需要
////
////    // 填充H矩阵右下块: At_inv (需要重新计算)
////    {
////        matrix_struct At_inv_new;
////        matrix_struct_zero_init(n, n, &At_inv_new);
////        matrix_struct_inv(&At, &At_inv_new);
////
////        for (int i = 0; i < n; i++) {
////            for (int j = 0; j < n; j++) {
////                H.data[n+i][n+j] = At_inv_new.data[i][j];
////            }
////        }
////        matrix_struct_free(&At_inv_new);
////        matrix_struct_free(&At); // 不再需要
////    }
////
////    // 阶段4：执行Schur分解（关键内存释放点）
////    matrix_struct T, U;
////    matrix_struct_zero_init(2*n, 2*n, &T);
////    matrix_struct_zero_init(2*n, 2*n, &U);
////
////    // 调用前确保所有可释放内存已释放
////    matrix_struct_schur(&H, &T, &U);
////    matrix_struct_free(&H); // 立即释放H
////
////    // 阶段5：提取U11和U21
////    matrix_struct U11, U21;
////    matrix_struct_zero_init(n, n, &U11);
////    matrix_struct_zero_init(n, n, &U21);
////
////    for (int i = 0; i < n; i++) {
////        for (int j = 0; j < n; j++) {
////            U11.data[i][j] = U.data[i][j];
////            U21.data[i][j] = U.data[n+i][j];
////        }
////    }
////    matrix_struct_free(&U); // 不再需要
////
////    // 阶段6：计算P = U21 * inv(U11)
////    matrix_struct U11_inv;
////    matrix_struct_zero_init(n, n, &U11_inv);
////    matrix_struct_inv(&U11, &U11_inv);
////    matrix_struct_mul(&U21, &U11_inv, P);
////
////    // 释放所有剩余临时矩阵
////    matrix_struct_free(&U11);
////    matrix_struct_free(&U21);
////    matrix_struct_free(&U11_inv);
////    matrix_struct_free(&T);
////
////    return 0;
////}
//
//int LQR_K_update_Schur(matrix_struct *Q, matrix_struct *R, matrix_struct *K)
//{
//    // 首先离散化系统（与原始代码相同）
//    matrix_struct Ad, Bd;
//    matrix_struct_zero_init(7, 7, &Ad);
//    matrix_struct_zero_init(7, 5, &Bd);
//
//    matrix_struct_exp(&A, 0.005f, &Ad);
//    matrix_struct_B_discetizing(&A, &B, 0.005f, &Bd);
//
//    // 使用 Schur 方法求解 DARE
//    matrix_struct P;
//    matrix_struct_zero_init(7, 7, &P);
//    solve_DARE_Schur(&Ad, &Bd, Q, R, &P);
//
//    // 计算最优增益矩阵 K
//    matrix_struct Bt, Rinv, BtP, BtPB, RplusBtPB, RplusBtPB_inv, BtPA;
//    matrix_struct_zero_init(5, 7, &Bt);
//    matrix_struct_zero_init(5, 5, &Rinv);
//    matrix_struct_zero_init(5, 7, &BtP);
//    matrix_struct_zero_init(5, 5, &BtPB);
//    matrix_struct_zero_init(5, 5, &RplusBtPB);
//    matrix_struct_zero_init(5, 5, &RplusBtPB_inv);
//    matrix_struct_zero_init(5, 7, &BtPA);
//
//    matrix_struct_tran(&Bd, &Bt);
//    matrix_struct_inv(R, &Rinv);
//    matrix_struct_mul(&Bt, &P, &BtP);
//    matrix_struct_mul(&BtP, &Bd, &BtPB);
//    matrix_struct_add(R, &BtPB, &RplusBtPB);
//    matrix_struct_inv(&RplusBtPB, &RplusBtPB_inv);
//    matrix_struct_mul(&BtP, &Ad, &BtPA);
//    matrix_struct_mul(&RplusBtPB_inv, &BtPA, K);
//
//    return 0; // 返回适当的值
//}
