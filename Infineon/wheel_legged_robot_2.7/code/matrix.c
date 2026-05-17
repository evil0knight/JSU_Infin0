/*
 * matrix.c
 *
 *  Created on: 2025年3月30日
 *      Author: 17104
 */

#include "matrix.h"

// 静态内存池
static float matrix_pool[MATRIX_POOL_SIZE];
static uint16_t pool_index = 0;

// 内存池初始化
void matrix_pool_init(void) {
    pool_index = 0;
}

// 矩阵内存分配
MatrixError matrix_alloc(uint8_t rows, uint8_t cols, Matrix* matrix) {
    if (!matrix || rows == 0 || cols == 0 ||
        rows > MAX_MATRIX_SIZE || cols > MAX_MATRIX_SIZE) {
        return MATRIX_INVALID_INPUT;
    }

    const size_t needed = rows * cols * sizeof(float);

    // 尝试静态分配
    if (rows * cols <= MAX_MATRIX_SIZE * MAX_MATRIX_SIZE) {
        matrix->is_static = true;
        matrix->rows = rows;
        matrix->cols = cols;
        return MATRIX_SUCCESS;
    }

    // 动态分配（从内存池）
    if (pool_index + needed > MATRIX_POOL_SIZE) {
        return MATRIX_ALLOC_FAILED;
    }

    matrix->is_static = false;
    matrix->rows = rows;
    matrix->cols = cols;
    matrix->data.dynamic_data = &matrix_pool[pool_index];
    pool_index += needed / sizeof(float);

    return MATRIX_SUCCESS;
}

// 矩阵内存释放
void matrix_free(Matrix* matrix) {
    if (matrix && !matrix->is_static) {
        // 动态分配的内存由内存池统一管理，不需要单独释放
        matrix->data.dynamic_data = NULL;
    }
    matrix->rows = 0;
    matrix->cols = 0;
}
//// 获取矩阵元素指针
//static inline float* matrix_data_ptr(Matrix* matrix) {
//    return matrix->is_static ? matrix->data.static_data : matrix->data.dynamic_data;
//}
//
//// 获取常量矩阵元素指针
//static inline const float* matrix_const_data_ptr(const Matrix* matrix) {
//    return matrix->is_static ? matrix->data.static_data : matrix->data.dynamic_data;
//}

// 检查矩阵有效性
bool matrix_is_valid(const Matrix* matrix) {
    return matrix && matrix->rows > 0 && matrix->cols > 0 &&
           ((matrix->is_static && matrix->rows <= MAX_MATRIX_SIZE && matrix->cols <= MAX_MATRIX_SIZE) ||
            (!matrix->is_static && matrix->data.dynamic_data));
}

bool matrix_is_square(const Matrix* matrix) {
    return matrix_is_valid(matrix) && (matrix->rows == matrix->cols);
}
int factorial(int num)
{
    int i, res = 1;
    for (i = 0; i < num - 1; i++)
    {
        res *= (num - i);
    }
    return res;
}
void matrix_show(const Matrix* matrix) {
    if (!matrix_is_valid(matrix)) {
        printf("Invalid matrix!\n");
        return;
    }

    printf("---show---\n");
    const float* data = matrix_const_data_ptr(matrix);

    for (uint8_t i = 0; i < matrix->rows; i++) {
        for (uint8_t j = 0; j < matrix->cols; j++) {
            printf("%.10f  ", data[i * matrix->cols + j]);
        }
        printf("\n");
    }
}
// 初始化全零矩阵
MatrixError matrix_zero_init(uint8_t rows, uint8_t cols, Matrix* matrix) {
    MatrixError err = matrix_alloc(rows, cols, matrix);
    if (err != MATRIX_SUCCESS) return err;

    float* data = matrix_data_ptr(matrix);
    memset(data, 0, rows * cols * sizeof(float));
    return MATRIX_SUCCESS;
}

// 初始化单位矩阵
MatrixError matrix_iden_init(uint8_t size, Matrix* matrix) {
    if (!matrix || size == 0 || (matrix->is_static && size > MAX_MATRIX_SIZE)) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_zero_init(size, size, matrix);
    if (err != MATRIX_SUCCESS) return err;

    float* data = matrix_data_ptr(matrix);
    const uint16_t step = size + 1;  // 对角线元素步长

    // 循环展开设置对角线元素
    uint8_t i = 0;
    for (; i + 3 < size; i += 4) {
        data[i * step] = 1.0f;           // data[i][i]
        data[(i+1) * step] = 1.0f;       // data[i+1][i+1]
        data[(i+2) * step] = 1.0f;       // data[i+2][i+2]
        data[(i+3) * step] = 1.0f;       // data[i+3][i+3]
    }
    // 处理剩余元素
    for (; i < size; i++) {
        data[i * step] = 1.0f;
    }
    return MATRIX_SUCCESS;
}
// 矩阵初始化（静态内存版）
MatrixError matrix_init(uint8_t row, uint8_t col, Matrix* mat, const float* data) {
    if (!mat || !data || row == 0 || col == 0 ||
        (mat->is_static && (row > MAX_MATRIX_SIZE || col > MAX_MATRIX_SIZE))) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_alloc(row, col, mat);
    if (err != MATRIX_SUCCESS) return err;

    float* dst = matrix_data_ptr(mat);
    const uint16_t total = row * col;

    // 使用memcpy加速批量复制
    if (total >= 16) {
        memcpy(dst, data, total * sizeof(float));
    } else {
        // 小矩阵手动展开
        for (uint8_t i = 0; i < total; i++) {
            dst[i] = data[i];
        }
    }
    return MATRIX_SUCCESS;
}
// 对角矩阵初始化
MatrixError matrix_diag_init(uint8_t row, Matrix* mat, const float* data) {
    if (!mat || !data || row == 0 ||
        (mat->is_static && row > MAX_MATRIX_SIZE)) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_zero_init(row, row, mat);
    if (err != MATRIX_SUCCESS) return err;

    float* mdata = matrix_data_ptr(mat);
    for (uint8_t i = 0; i < row; i++) {
        mdata[i * row + i] = data[i];
    }
    return MATRIX_SUCCESS;
}
// 矩阵复制
MatrixError matrix_copy(const Matrix* src, Matrix* dest) {
    if (!matrix_is_valid(src) || !dest) {
        return MATRIX_INVALID_INPUT;
    }

    dest->rows = src->rows;
    dest->cols = src->cols;
    dest->is_static = src->is_static;

    const uint16_t total = src->rows * src->cols;
    const float* sdata = matrix_const_data_ptr(src);
    float* ddata = matrix_data_ptr(dest);

    // 根据矩阵大小选择复制策略
    if (total >= 32) {
        // 大矩阵使用memcpy
        memcpy(ddata, sdata, total * sizeof(float));
    } else {
        // 小矩阵手动展开复制
        uint16_t i = 0;
        for (; i + 3 < total; i += 4) {
            ddata[i]   = sdata[i];
            ddata[i+1] = sdata[i+1];
            ddata[i+2] = sdata[i+2];
            ddata[i+3] = sdata[i+3];
        }
        // 处理剩余元素
        for (; i < total; i++) {
            ddata[i] = sdata[i];
        }
    }
    return MATRIX_SUCCESS;
}
// 矩阵加法 (极致优化版)
MatrixError matrix_add(const Matrix* a, const Matrix* b, Matrix* result) {
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !result ||
        a->rows != b->rows || a->cols != b->cols) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_alloc(a->rows, a->cols, result);
    if (err != MATRIX_SUCCESS) return err;

    const float* adata = matrix_const_data_ptr(a);
    const float* bdata = matrix_const_data_ptr(b);
    float* rdata = matrix_data_ptr(result);
    const uint16_t total = a->rows * a->cols;

    // ARM Cortex-M4汇编级优化思路的手写C代码
    uint16_t i = 0;
    for (; i + 3 < total; i += 4) {
        // 使用并行计算思想
        rdata[i]   = adata[i]   + bdata[i];
        rdata[i+1] = adata[i+1] + bdata[i+1];
        rdata[i+2] = adata[i+2] + bdata[i+2];
        rdata[i+3] = adata[i+3] + bdata[i+3];
    }
    // 处理剩余元素
    for (; i < total; i++) {
        rdata[i] = adata[i] + bdata[i];
    }

    return MATRIX_SUCCESS;
}
// 矩阵减法（优化版）
MatrixError matrix_sub(const Matrix* a, const Matrix* b, Matrix* result) {
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !result ||
        a->rows != b->rows || a->cols != b->cols) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_alloc(a->rows, a->cols, result);
    if (err != MATRIX_SUCCESS) return err;

    const float* adata = matrix_const_data_ptr(a);
    const float* bdata = matrix_const_data_ptr(b);
    float* rdata = matrix_data_ptr(result);
    const uint16_t total = a->rows * a->cols;

    // 8路循环展开
    uint16_t i = 0;
    for (; i + 7 < total; i += 8) {
        rdata[i]   = adata[i]   - bdata[i];
        rdata[i+1] = adata[i+1] - bdata[i+1];
        rdata[i+2] = adata[i+2] - bdata[i+2];
        rdata[i+3] = adata[i+3] - bdata[i+3];
        rdata[i+4] = adata[i+4] - bdata[i+4];
        rdata[i+5] = adata[i+5] - bdata[i+5];
        rdata[i+6] = adata[i+6] - bdata[i+6];
        rdata[i+7] = adata[i+7] - bdata[i+7];
    }
    // 处理剩余元素
    for (; i < total; i++) {
        rdata[i] = adata[i] - bdata[i];
    }
    return MATRIX_SUCCESS;
}

// 矩阵乘法 (极致优化版)
MatrixError matrix_mul(const Matrix* a, const Matrix* b, Matrix* result) {
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !result ||
        a->cols != b->rows) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t m = a->rows, n = a->cols, p = b->cols;
    MatrixError err = matrix_alloc(m, p, result);
    if (err != MATRIX_SUCCESS) return err;

    const float* a_data = matrix_const_data_ptr(a);
    const float* b_data = matrix_const_data_ptr(b);
    float* r_data = matrix_data_ptr(result);
    memset(r_data, 0, m * p * sizeof(float));

    // 高度优化的乘法核心
    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t k = 0; k < n; k++) {
            register const float temp = a_data[i * n + k];  // 使用寄存器变量
            register float* r_row = &r_data[i * p];
            register const float* b_row = &b_data[k * p];

            // 手动展开内层循环
            uint8_t j = 0;
            for (; j + 3 < p; j += 4) {
                r_row[j]   += temp * b_row[j];
                r_row[j+1] += temp * b_row[j+1];
                r_row[j+2] += temp * b_row[j+2];
                r_row[j+3] += temp * b_row[j+3];
            }
            // 处理剩余元素
            for (; j < p; j++) {
                r_row[j] += temp * b_row[j];
            }
        }
    }

    return MATRIX_SUCCESS;
}

// 标量乘法（优化版）
MatrixError matrix_scalar_mul(const Matrix* a, float scalar, Matrix* result) {
    if (!matrix_is_valid(a) || !result) {
        return MATRIX_INVALID_INPUT;
    }

    MatrixError err = matrix_alloc(a->rows, a->cols, result);
    if (err != MATRIX_SUCCESS) return err;

    const float* a_data = matrix_const_data_ptr(a);
    float* r_data = matrix_data_ptr(result);
    const uint16_t total = a->rows * a->cols;

    // 循环展开优化
    uint16_t i = 0;
    for (; i + 3 < total; i += 4) {
        r_data[i]   = a_data[i]   * scalar;
        r_data[i+1] = a_data[i+1] * scalar;
        r_data[i+2] = a_data[i+2] * scalar;
        r_data[i+3] = a_data[i+3] * scalar;
    }
    // 处理剩余元素
    for (; i < total; i++) {
        r_data[i] = a_data[i] * scalar;
    }
    return MATRIX_SUCCESS;
}
// 矩阵转置（内存优化版）
MatrixError matrix_transpose(const Matrix* src, Matrix* result) {
    if (!matrix_is_valid(src) || !result) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t m = src->rows, n = src->cols;
    MatrixError err = matrix_alloc(n, m, result);
    if (err != MATRIX_SUCCESS) return err;

    const float* sdata = matrix_const_data_ptr(src);
    float* rdata = matrix_data_ptr(result);

    // 分块转置优化（提高缓存命中率）
    const uint8_t block_size = 4;
    for (uint8_t i = 0; i < m; i += block_size) {
        for (uint8_t j = 0; j < n; j += block_size) {
            // 处理分块
            const uint8_t i_end = (i + block_size) < m ? (i + block_size) : m;
            const uint8_t j_end = (j + block_size) < n ? (j + block_size) : n;

            for (uint8_t ii = i; ii < i_end; ii++) {
                for (uint8_t jj = j; jj < j_end; jj++) {
                    rdata[jj * m + ii] = sdata[ii * n + jj];
                }
            }
        }
    }
    return MATRIX_SUCCESS;
}
// 行列式计算（基于LU分解）
float matrix_det(const Matrix* mat) {
    if (!matrix_is_square(mat)) return NAN;

    const uint8_t n = mat->rows;
    // 小矩阵直接使用公式计算
    if (n == 1) return *(matrix_const_data_ptr(mat));
    if (n == 2) {
        const float* d = matrix_const_data_ptr(mat);
        return d[0]*d[3] - d[1]*d[2];
    }
    if (n == 3) {
        const float* d = matrix_const_data_ptr(mat);
        return d[0]*(d[4]*d[8] - d[5]*d[7]) -
               d[1]*(d[3]*d[8] - d[5]*d[6]) +
               d[2]*(d[3]*d[7] - d[4]*d[6]);
    }

    // 大矩阵使用LU分解
    Matrix L, U;
    if (lu_decomposition(mat, &L, &U) != MATRIX_SUCCESS) {
        return NAN;
    }

    float det = 1.0f;
    const float* u_data = matrix_const_data_ptr(&U);

    // 计算对角线乘积
    for (uint8_t i = 0; i < n; i++) {
        det *= u_data[i * n + i];
    }

    // 行列式符号由置换次数决定，简化处理
    // 实际实现可能需要跟踪行交换次数
    matrix_free(&L);
    matrix_free(&U);

    return det;
}
// 矩阵迹
float matrix_trace(const Matrix* mat) {
    if (!matrix_is_square(mat)) return NAN;

    const uint8_t n = mat->rows;
    const float* data = matrix_const_data_ptr(mat);
    float tr = 0.0f;

    // 循环展开计算迹
    uint8_t i = 0;
    for (; i + 3 < n; i += 4) {
        tr += data[i * n + i];
        tr += data[(i+1) * n + (i+1)];
        tr += data[(i+2) * n + (i+2)];
        tr += data[(i+3) * n + (i+3)];
    }
    // 处理剩余对角线元素
    for (; i < n; i++) {
        tr += data[i * n + i];
    }
    return tr;
}

// Frobenius范数
float matrix_frobenius_norm(const Matrix* mat) {
    if (!matrix_is_valid(mat)) return NAN;

    const uint16_t total = mat->rows * mat->cols;
    const float* data = matrix_const_data_ptr(mat);
    float sum = 0.0f;

    // 使用Kahan求和算法提高精度
    float compensation = 0.0f;
    for (uint16_t i = 0; i < total; i++) {
        float val = data[i] * data[i] - compensation;
        float temp = sum + val;
        compensation = (temp - sum) - val;
        sum = temp;
    }
    return sqrtf(sum);
}

MatrixError matrix_exp(Matrix* A, float Ts, Matrix* res) {
    // 参数检查
    if (!matrix_is_square(A) || !res) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t n = A->rows;
    Matrix term, sum, Ak, temp;
    float factorial = 1.0f;
    float Ts_pow = Ts;

    // 预分配所有矩阵内存
    MatrixError err = matrix_iden_init(n, &sum);  // sum = I
    if (err != MATRIX_SUCCESS) return err;

    err = matrix_iden_init(n, &term); // term = I
    if (err != MATRIX_SUCCESS) {
        matrix_free(&sum);
        return err;
    }

    err = matrix_zero_init(n, n, &Ak);
    if (err != MATRIX_SUCCESS) {
        matrix_free(&sum);
        matrix_free(&term);
        return err;
    }

    err = matrix_zero_init(n, n, &temp);
    if (err != MATRIX_SUCCESS) {
        matrix_free(&sum);
        matrix_free(&term);
        matrix_free(&Ak);
        return err;
    }

    // 泰勒级数展开 (优化计算)
    for (uint8_t k = 1; k <= 10; k++) { // 减少迭代次数，10次通常足够
        // term = term * A * Ts / k
        err = matrix_mul(&term, A, &temp);
        if (err != MATRIX_SUCCESS) break;

        matrix_copy(&temp, &term);

        const float scale = Ts_pow / factorial;
        err = matrix_scalar_mul(&term, scale, &temp);
        if (err != MATRIX_SUCCESS) break;

        matrix_copy(&temp, &term);

        // sum += term
        err = matrix_add(&sum, &term, &temp);
        if (err != MATRIX_SUCCESS) break;

        matrix_copy(&temp, &sum);

        // 更新中间变量
        Ts_pow *= Ts;
        factorial *= (k + 1);

        // 提前终止检查 (当term足够小时)
        float norm = matrix_frobenius_norm(&term);
        if (norm < 1e-10f) break;
    }

    // 复制结果
    if (err == MATRIX_SUCCESS) {
        err = matrix_copy(&sum, res);
    }

    // 释放临时矩阵
    matrix_free(&term);
    matrix_free(&sum);
    matrix_free(&Ak);
    matrix_free(&temp);

    return err;
}
// 前代法（优化版）
static MatrixError forward_substitution(const Matrix* L, Matrix* y, const Matrix* b) {
    if (!matrix_is_valid(L) || !y || !matrix_is_valid(b) ||
        !matrix_is_square(L) || L->rows != b->rows || y->rows != b->rows) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t n = L->rows;
    const float* L_data = matrix_const_data_ptr(L);
    const float* b_data = matrix_const_data_ptr(b);
    float* y_data = matrix_data_ptr(y);

    // 复制b到y
    memcpy(y_data, b_data, n * sizeof(float));

    for (uint8_t i = 0; i < n; i++) {
        // 预计算1/L[i][i]
        const float inv_Lii = 1.0f / L_data[i * n + i];

        // 更新当前行
        y_data[i] *= inv_Lii;

        // 更新后续行
        for (uint8_t k = i + 1; k < n; k++) {
            y_data[k] -= L_data[k * n + i] * y_data[i];
        }
    }
    return MATRIX_SUCCESS;
}
// 回代法（优化版）
static MatrixError backward_substitution(const Matrix* U, Matrix* x, const Matrix* y) {
    if (!matrix_is_valid(U) || !x || !matrix_is_valid(y) ||
        !matrix_is_square(U) || U->rows != y->rows || x->rows != y->rows) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t n = U->rows;
    const float* U_data = matrix_const_data_ptr(U);
    const float* y_data = matrix_const_data_ptr(y);
    float* x_data = matrix_data_ptr(x);

    // 复制y到x
    memcpy(x_data, y_data, n * sizeof(float));

    for (int8_t i = n - 1; i >= 0; i--) {
        // 预计算1/U[i][i]
        const float inv_Uii = 1.0f / U_data[i * n + i];

        // 更新当前行
        x_data[i] *= inv_Uii;

        // 更新前续行
        for (int8_t k = i - 1; k >= 0; k--) {
            x_data[k] -= U_data[k * n + i] * x_data[i];
        }
    }
    return MATRIX_SUCCESS;
}
// LU分解 (优化版)
static MatrixError lu_decomposition(const Matrix* A, Matrix* L, Matrix* U) {
    if (!matrix_is_square(A) || !L || !U) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t n = A->rows;
    MatrixError err = matrix_copy(A, U);
    if (err != MATRIX_SUCCESS) return err;

    err = matrix_iden_init(n, L);
    if (err != MATRIX_SUCCESS) return err;

    const float epsilon = 1e-12f;
    float* L_data = matrix_data_ptr(L);
    float* U_data = matrix_data_ptr(U);

    for (uint8_t k = 0; k < n; k++) {
        if (fabsf(U_data[k * n + k]) < epsilon) {
            return MATRIX_SINGULAR;
        }

        // 预计算1/U[k][k]避免重复除法
        const float inv_diag = 1.0f / U_data[k * n + k];

        for (uint8_t i = k + 1; i < n; i++) {
            L_data[i * n + k] = U_data[i * n + k] * inv_diag;

            // 手动展开k循环
            uint8_t j = k;
            for (; j + 3 < n; j += 4) {
                U_data[i * n + j]   -= L_data[i * n + k] * U_data[k * n + j];
                U_data[i * n + j+1] -= L_data[i * n + k] * U_data[k * n + j+1];
                U_data[i * n + j+2] -= L_data[i * n + k] * U_data[k * n + j+2];
                U_data[i * n + j+3] -= L_data[i * n + k] * U_data[k * n + j+3];
            }
            // 处理剩余元素
            for (; j < n; j++) {
                U_data[i * n + j] -= L_data[i * n + k] * U_data[k * n + j];
            }
        }
    }
    return MATRIX_SUCCESS;
}
// 矩阵求逆 (极致优化版)
MatrixError matrix_inverse(const Matrix* src, Matrix* result) {
    if (!matrix_is_square(src) || !result) {
        return MATRIX_INVALID_INPUT;
    }

    const uint8_t n = src->rows;
    Matrix L, U, I, y, x_col;
    MatrixError err = matrix_iden_init(n, &I);
    if (err != MATRIX_SUCCESS) return err;

    err = matrix_zero_init(n, n, result);
    if (err != MATRIX_SUCCESS) {
        matrix_free(&I);
        return err;
    }

    // LU分解
    err = lu_decomposition(src, &L, &U);
    if (err != MATRIX_SUCCESS) {
        matrix_free(&I);
        matrix_free(result);
        return err;
    }

    // 对每一列求解
    for (uint8_t col = 0; col < n; col++) {
        Matrix b;
        err = matrix_zero_init(n, 1, &b);
        if (err != MATRIX_SUCCESS) break;

        float* b_data = matrix_data_ptr(&b);
        b_data[col] = 1.0f;

        err = matrix_zero_init(n, 1, &y);
        if (err != MATRIX_SUCCESS) {
            matrix_free(&b);
            break;
        }

        err = forward_substitution(&L, &y, &b);
        if (err != MATRIX_SUCCESS) {
            matrix_free(&b);
            matrix_free(&y);
            break;
        }

        err = matrix_zero_init(n, 1, &x_col);
        if (err != MATRIX_SUCCESS) {
            matrix_free(&b);
            matrix_free(&y);
            break;
        }

        err = backward_substitution(&U, &x_col, &y);
        if (err == MATRIX_SUCCESS) {
            float* res_data = matrix_data_ptr(result);
            const float* x_data = matrix_const_data_ptr(&x_col);
            for (uint8_t i = 0; i < n; i++) {
                res_data[i * n + col] = x_data[i];
            }
        }

        // 释放临时矩阵
        matrix_free(&b);
        matrix_free(&y);
        matrix_free(&x_col);

        if (err != MATRIX_SUCCESS) break;
    }

    // 释放资源
    matrix_free(&L);
    matrix_free(&U);
    matrix_free(&I);

    return err;
}
