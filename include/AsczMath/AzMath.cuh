#ifndef AZMATH_CUH
#define AZMATH_CUH

#include <Vector.cuh>

class AzMath {
public:
    __device__ static float rsqrt(float x) {
        float xhalf = 0.5f * x;
        int i = *(int*)&x;
        i = 0x5f3759df - (i >> 1);
        x = *(float*)&i;
        x = x * (1.5f - xhalf * x * x);
        return x;
    }
};

#endif