#ifndef AZDEVMATH_CUH
#define AZDEVMATH_CUH

// A class that contain optimized math functions for device
class AzDevMath {
public:
    // Fast inverse square root (Quake 3 method)
    __device__ static float rsqrt(float x) {
        float xhalf = 0.5f * x;
        int i = *(int*)&x;
        i = 0x5f3759df - (i >> 1);
        x = *(float*)&i;
        x = x * (1.5f - xhalf * x * x);
        return x;
    }

    // ACES Filmic Tone Mapping
    __device__ static float ACESFilm(float x) {
        return (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f);
    }

    // Fast exponential approximation (GPU-friendly)
    __device__ static float exp(float x) {
        x = 1.0f + x / 256.0f;
        x *= x; x *= x; x *= x; x *= x;
        x *= x; x *= x; x *= x; x *= x;
        return x;
    }

    // Fast natural log approximation (GPU-friendly)
    __device__ static float log(float x) {
        int i = *(int*)&x;
        float y = i;
        y = y * 1.1920928955078125e-7f - 126.94269504f; // Magic constant for log2
        return y * 0.69314718f; // Convert log2 to natural log
    }

    // Fast Approximation of sin(x) using a quartic polynomial
    __device__ static float sin(float x) {
        const float B = 4.0f / M_PI;
        const float C = -4.0f / (M_PI * M_PI);
        float y = B * x + C * x * fabsf(x);
        return 0.775f * y + 0.225f * y * fabsf(y);
    }

    // Fast Approximation of cos(x) using phase-shifted fastSin
    __device__ static float cos(float x) {
        return AzDevMath::sin(x + M_PI * 0.5f);
    }

    // Wang Hash (for procedural randomness in CUDA kernels)
    __device__ static unsigned int wangHash(unsigned int seed) {
        seed = (seed ^ 61) ^ (seed >> 16);
        seed *= 9;
        seed ^= seed >> 4;
        seed *= 0x27d4eb2d;
        seed ^= seed >> 15;
        return seed;
    }

    // Spatial Hashing (for BVH, grid-based acceleration structures)
    __device__ static unsigned int spatialHash(unsigned int x, unsigned int y, unsigned int z) {
        const unsigned int PRIME1 = 73856093;
        const unsigned int PRIME2 = 19349663;
        const unsigned int PRIME3 = 83492791;
        return (x * PRIME1) ^ (y * PRIME2) ^ (z * PRIME3);
    }

    // Cosine-weighted importance sampling (for diffuse BRDFs)
    __device__ static void cosineSampleHemisphere(float u1, float u2, float& x, float& y, float& z) {
        float r = sqrtf(u1);
        float theta = 2.0f * M_PI * u2;
        x = r * cosf(theta);
        y = r * sinf(theta);
        z = sqrtf(1.0f - u1);
    }

    // Uniform sphere sampling (for environment lighting)
    __device__ static void uniformSampleSphere(float u1, float u2, float& x, float& y, float& z) {
        float z_val = 1.0f - 2.0f * u1;
        float r = sqrtf(fmaxf(0.0f, 1.0f - z_val * z_val));
        float phi = 2.0f * M_PI * u2;
        x = r * cosf(phi);
        y = r * sinf(phi);
        z = z_val;
    }

    // Low-discrepancy sequence (Sobol Sequence for Quasi-Monte Carlo integration)
    __device__ static float sobol(int index, int dimension) {
        unsigned int v = 1u << (31 - __clz(index + 1));
        unsigned int result = 0;
        for (int i = 0; i < 32; i++) {
            if (index & (1 << i))
                result ^= (v >> i);
        }
        return result * (1.0f / 4294967296.0f); // Normalize to [0,1]
    }

    // Fibonacci Spiral Sampling (for even hemisphere distribution)
    __device__ static void fibonacciSphereSample(int i, int n, float& x, float& y, float& z) {
        float phi = M_PI * (3.0f - sqrtf(5.0f));  // Golden angle
        float y_pos = 1.0f - (i / float(n - 1)) * 2.0f;
        float radius = sqrtf(1.0f - y_pos * y_pos);
        float theta = phi * i;
        x = cosf(theta) * radius;
        y = y_pos;
        z = sinf(theta) * radius;
    }

    // Smith GGX Shadowing Function (for Microfacet BRDFs)
    __device__ static float smithG1(float NoV, float alpha) {
        float a = alpha * alpha;
        float b = NoV * NoV;
        return 2.0f / (1.0f + sqrtf(1.0f + a * (1.0f - b) / b));
    }

    // GGX Normal Distribution Function (for roughness in path tracing)
    __device__ static float ggxNDF(float NoH, float alpha) {
        float a2 = alpha * alpha;
        float d = (NoH * NoH) * (a2 - 1.0f) + 1.0f;
        return a2 / (M_PI * d * d);
    }
};

#endif