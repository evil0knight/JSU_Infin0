/*
 * matrix.h
 *
 *  Created on: 2025骞�3鏈�30鏃�
 *      Author: 17104
 */

#ifndef CODE_MATRIX_H_
#define CODE_MATRIX_H_

#include "zf_common_headfile.h"

// 错误代码定义
typedef enum {
    MATRIX_SUCCESS = 0,
    MATRIX_INVALID_INPUT = -1,
    MATRIX_SIZE_MISMATCH = -2,
    MATRIX_ALLOC_FAILED = -3,
    MATRIX_SINGULAR = -4,      // 矩阵奇异
    MATRIX_NOT_SQUARE = -5,
    MATRIX_DIM_TOO_LARGE = -6,
    MATRIX_NUMERICAL_ERROR = -7
} MatrixError;

// 系统配置
#ifndef MAX_MATRIX_SIZE
#define MAX_MATRIX_SIZE 8  // 根据TC377内存调整
#endif

// 内存池配置
#define MATRIX_POOL_SIZE (MAX_MATRIX_SIZE * MAX_MATRIX_SIZE * 4) // 预分配内存池

// 矩阵结构（支持静态和动态分配）
typedef struct {
    uint8_t rows;
    uint8_t cols;
    bool is_static;  // 标记是否为静态分配
    union {
        float* dynamic_data;  // 动态分配指针
        float static_data[MAX_MATRIX_SIZE * MAX_MATRIX_SIZE]; // 静态分配内存
    } data;
} Matrix;

// 内存管理
void matrix_pool_init(void);
MatrixError matrix_alloc(uint8_t rows, uint8_t cols, Matrix* matrix);
void matrix_free(Matrix* matrix);
// 获取矩阵元素指针
static inline float* matrix_data_ptr(Matrix* matrix) {
    return matrix->is_static ? matrix->data.static_data : matrix->data.dynamic_data;
}
// 获取常量矩阵元素指针
static inline const float* matrix_const_data_ptr(const Matrix* matrix) {
    return matrix->is_static ? matrix->data.static_data : matrix->data.dynamic_data;
}

// 初始化函数
MatrixError matrix_zero_init(uint8_t rows, uint8_t cols, Matrix* matrix);
MatrixError matrix_iden_init(uint8_t size, Matrix* matrix);
MatrixError matrix_init(uint8_t row, uint8_t column, Matrix* matrix, const float* data);
MatrixError matrix_diag_init(uint8_t row, Matrix* matrix, const float* data);

// 矩阵运算
MatrixError matrix_copy(const Matrix* src, Matrix* dest);
MatrixError matrix_add(const Matrix* a, const Matrix* b, Matrix* result);
MatrixError matrix_sub(const Matrix* a, const Matrix* b, Matrix* result);
MatrixError matrix_mul(const Matrix* a, const Matrix* b, Matrix* result);
MatrixError matrix_scalar_mul(const Matrix* a, float scalar, Matrix* result);
MatrixError matrix_transpose(const Matrix* src, Matrix* result);

// 高级运算
MatrixError matrix_inverse(const Matrix* src, Matrix* result);
MatrixError matrix_exp(Matrix* A, float Ts, Matrix* res);
float matrix_det(const Matrix* matrix);
float matrix_trace(const Matrix* matrix);
float matrix_frobenius_norm(const Matrix* matrix);

// 辅助函数
bool matrix_is_valid(const Matrix* matrix);
bool matrix_is_square(const Matrix* matrix);
int factorial(int num);
void matrix_show(const Matrix* matrix);

// 内部函数声明
static MatrixError lu_decomposition(const Matrix* A, Matrix* L, Matrix* U);
static MatrixError forward_substitution(const Matrix* L, Matrix* y, const Matrix* b);
static MatrixError backward_substitution(const Matrix* U, Matrix* x, const Matrix* y);

#endif /* CODE_MATRIX_H_ */
