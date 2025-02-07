#ifndef MATRIX_CUH
#define MATRIX_CUH

#include <Vector.cuh>

struct Mat4 {
    float data[4][4] = {0};
    _hst_dev_ Mat4();
    _hst_dev_ Mat4(float data[4][4]);

    // Basic operations
    _hst_dev_ Mat4 operator+(const Mat4 &mat);
    _hst_dev_ Mat4 operator-(const Mat4 &mat);
    _hst_dev_ Mat4 operator*(const float scl);

    // Advanced operations
    _hst_dev_ Flt4 operator*(const Flt4 &vec);
    _hst_dev_ Mat4 operator*(const Mat4 &mat);
    _hst_dev_ float det(); // Determinant
};

#endif