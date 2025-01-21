#ifndef MATRIX_CUH
#define MATRIX_CUH

#include <Vector.cuh>

struct Mat4f {
    float data[4][4] = {0};
    _hst_dev_ Mat4f();
    _hst_dev_ Mat4f(float data[4][4]);

    // Basic operations
    _hst_dev_ Mat4f operator+(const Mat4f &mat);
    _hst_dev_ Mat4f operator-(const Mat4f &mat);
    _hst_dev_ Mat4f operator*(const float scl);
    // Advanced operations
    _hst_dev_ Vec4f operator*(const Vec4f &vec);
    _hst_dev_ Mat4f operator*(const Mat4f &mat);
    _hst_dev_ float det(); // Determinant
};

#endif