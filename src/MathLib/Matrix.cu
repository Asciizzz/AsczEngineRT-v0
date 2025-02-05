#include <Matrix.cuh>

Mat4f::Mat4f() {
    // Identity matrix
    data[0][0] = 1;
    data[1][1] = 1;
    data[2][2] = 1;
    data[3][3] = 1;
}

Mat4f::Mat4f(float data[4][4]) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            this->data[i][j] = data[i][j];
        }
    }
}

Mat4f Mat4f::operator+(const Mat4f &mat) {
    Mat4f result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.data[i][j] = data[i][j] + mat.data[i][j];
        }
    }

    return result;
}

Mat4f Mat4f::operator-(const Mat4f &mat) {
    Mat4f result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.data[i][j] = data[i][j] - mat.data[i][j];
        }
    }

    return result;
}

Mat4f Mat4f::operator*(const float scl) {
    Mat4f result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.data[i][j] = data[i][j] * scl;
        }
    }

    return result;
}

Flt4 Mat4f::operator*(const Flt4 &vec) {
    Flt4 result;
    result.x = data[0][0] * vec.x + data[0][1] * vec.y + data[0][2] * vec.z + data[0][3] * vec.w;
    result.y = data[1][0] * vec.x + data[1][1] * vec.y + data[1][2] * vec.z + data[1][3] * vec.w;
    result.z = data[2][0] * vec.x + data[2][1] * vec.y + data[2][2] * vec.z + data[2][3] * vec.w;
    result.w = data[3][0] * vec.x + data[3][1] * vec.y + data[3][2] * vec.z + data[3][3] * vec.w;

    return result;
}

Mat4f Mat4f::operator*(const Mat4f &mat) {
    Mat4f result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.data[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                result.data[i][j] += data[i][k] * mat.data[k][j];
            }
        }
    }

    return result;
}

float Mat4f::det() {
    float detA11 = data[1][1] * (data[2][2] * data[3][3] - data[2][3] * data[3][2]) - 
                   data[1][2] * (data[2][1] * data[3][3] - data[2][3] * data[3][1]) + 
                   data[1][3] * (data[2][1] * data[3][2] - data[2][2] * data[3][1]);

    float detA12 = data[1][0] * (data[2][2] * data[3][3] - data[2][3] * data[3][2]) - 
                   data[1][2] * (data[2][0] * data[3][3] - data[2][3] * data[3][0]) + 
                   data[1][3] * (data[2][0] * data[3][2] - data[2][2] * data[3][0]);

    float detA13 = data[1][0] * (data[2][1] * data[3][3] - data[2][3] * data[3][1]) - 
                   data[1][1] * (data[2][0] * data[3][3] - data[2][3] * data[3][0]) + 
                   data[1][3] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]);

    float detA14 = data[1][0] * (data[2][1] * data[3][2] - data[2][2] * data[3][1]) - 
                   data[1][1] * (data[2][0] * data[3][2] - data[2][2] * data[3][0]) + 
                   data[1][2] * (data[2][0] * data[3][1] - data[2][1] * data[3][0]);

    // Cofactor expansion along the first row
    float a = data[0][0], b = data[0][1], c = data[0][2], d = data[0][3];

    return a * detA11 - b * detA12 + c * detA13 - d * detA14;
}
