#include <Vector.cuh>
#include <Matrix.cuh>

// VEC2f
Vec2f::Vec2f() : x(0), y(0) {}
Vec2f::Vec2f(float x, float y) : x(x), y(y) {}
Vec2f Vec2f::operator+(const Vec2f& v) {
    return Vec2f(x + v.x, y + v.y);
}
Vec2f Vec2f::operator+(const float t) {
    return Vec2f(x + t, y + t);
}
Vec2f Vec2f::operator-(const Vec2f& v) {
    return Vec2f(x - v.x, y - v.y);
}
Vec2f Vec2f::operator-(const float t) {
    return Vec2f(x - t, y - t);
}
Vec2f Vec2f::operator*(const float scl) {
    return Vec2f(x * scl, y * scl);
}
Vec2f Vec2f::operator/(const float scl) {
    return Vec2f(x / scl, y / scl);
}

// VEC3f
Vec3f::Vec3f() : x(0), y(0), z(0) {}
Vec3f::Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
Vec3f::Vec3f(float a) : x(a), y(a), z(a) {}
Vec4f Vec3f::toVec4f() {
    return Vec4f(x, y, z, 1);
}

Vec3f Vec3f::operator+(const Vec3f& v) const {
    return Vec3f(x + v.x, y + v.y, z + v.z);
}
Vec3f Vec3f::operator+(const float t) const {
    return Vec3f(x + t, y + t, z + t);
}
Vec3f Vec3f::operator-(const Vec3f& v) const {
    return Vec3f(x - v.x, y - v.y, z - v.z);
}
Vec3f Vec3f::operator-(const float t) const {
    return Vec3f(x - t, y - t, z - t);
}
Vec3f Vec3f::operator*(const float scl) const {
    return Vec3f(x * scl, y * scl, z * scl);
}
Vec3f Vec3f::operator/(const float scl) const {
    return Vec3f(x / scl, y / scl, z / scl);
}
void Vec3f::operator+=(const Vec3f& v) {
    x += v.x; y += v.y; z += v.z;
}
void Vec3f::operator-=(const Vec3f& v) {
    x -= v.x; y -= v.y; z -= v.z;
}
void Vec3f::operator*=(const float scl) {
    x *= scl; y *= scl; z *= scl;
}
void Vec3f::operator/=(const float scl) {
    x /= scl; y /= scl; z /= scl;
}

float Vec3f::operator*(const Vec3f& v) const {
    return x * v.x + y * v.y + z * v.z;
}
Vec3f Vec3f::operator&(const Vec3f& v) const {
    return Vec3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}
float Vec3f::mag() {
    return sqrt(x * x + y * y + z * z);
}
void Vec3f::norm() {
    float m = mag();
    x /= m; y /= m; z /= m;
}

Vec3f Vec3f::bary(Vec2f v, Vec2f v0, Vec2f v1, Vec2f v2) {
    float d = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
    float a = ((v1.y - v2.y) * (v.x - v2.x) + (v2.x - v1.x) * (v.y - v2.y)) / d;
    float b = ((v2.y - v0.y) * (v.x - v2.x) + (v0.x - v2.x) * (v.y - v2.y)) / d;
    float c = 1 - a - b;
    return Vec3f(a, b, c);
}
void Vec3f::limit(float min, float max) {
    x = std::max(min, std::min(x, max));
    y = std::max(min, std::min(y, max));
    z = std::max(min, std::min(z, max));
}

// Transformations
Vec3f Vec3f::translate(Vec3f& v, const Vec3f& t) {
    return v + t;
}

Vec3f Vec3f::rotate(Vec3f &v, const Vec3f &o, const Vec3f &n, const float w) {
    // Note: the rodriques rotation is based on the right hand rule

    Vec3f dlt = v - o;
    Vec4f dlt4 = dlt.toVec4f();

    float cosW = cos(w), sinW = sin(w);

    // The Rodrigues formula
    Vec3f p = dlt*cosW + (n & v)*sinW + n*(n * v)*(1 - cosW);
    p += o;

    return p;
}

Vec3f Vec3f::rotateX(Vec3f &v, const Vec3f &o, const float rx) {
    Vec3f dlt = v - o;
    Vec4f dlt4 = dlt.toVec4f();

    float cosX = cos(rx), sinX = sin(rx);
    float rX[4][4] = {
        {1, 0, 0, 0},
        {0, cosX, -sinX, 0},
        {0, sinX, cosX, 0},
        {0, 0, 0, 1}
    };

    Vec4f rVec4 = Mat4f(rX) * dlt4;
    Vec3f rVec3 = rVec4.toVec3f();
    rVec3 += o;

    return rVec3;
}
Vec3f Vec3f::rotateY(Vec3f &v, const Vec3f &o, const float ry) {
    Vec3f dlt = v - o;
    Vec4f dlt4 = dlt.toVec4f();

    float cosY = cos(ry), sinY = sin(ry);
    float rY[4][4] = {
        {cosY, 0, sinY, 0},
        {0, 1, 0, 0},
        {-sinY, 0, cosY, 0},
        {0, 0, 0, 1}
    };

    Vec4f rVec4 = Mat4f(rY) * dlt4;
    Vec3f rVec3 = rVec4.toVec3f();
    rVec3 += o;

    return rVec3;
}
Vec3f Vec3f::rotateZ(Vec3f &v, const Vec3f &o, const float rz) {
    Vec3f dlt = v - o;
    Vec4f dlt4 = dlt.toVec4f();

    float cosZ = cos(rz), sinZ = sin(rz);
    float rZ[4][4] = {
        {cosZ, -sinZ, 0, 0},
        {sinZ, cosZ, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    Vec4f rVec4 = Mat4f(rZ) * dlt4;
    Vec3f rVec3 = rVec4.toVec3f();
    rVec3 += o;

    return rVec3;
}

Vec3f Vec3f::scale(Vec3f& v, const Vec3f& o, const Vec3f& scl) {
    Vec3f dlt = v - o;
    return Vec3f(
        o.x + dlt.x * scl.x,
        o.y + dlt.y * scl.y,
        o.z + dlt.z * scl.z
    );
}
Vec3f Vec3f::scale(Vec3f& v, const Vec3f& o, const float scl) {
    return scale(v, o, Vec3f(scl));
}

// Transformations but on self
void Vec3f::translate(const Vec3f& t) {
    *this += t;
}

void Vec3f::rotateX(const Vec3f& o, const float rx) {
    *this = rotateX(*this, o, rx);
}
void Vec3f::rotateY(const Vec3f& o, const float ry) {
    *this = rotateY(*this, o, ry);
}
void Vec3f::rotateZ(const Vec3f& o, const float rz) {
    *this = rotateZ(*this, o, rz);
}

void Vec3f::scale(const Vec3f& o, const Vec3f& scl) {
    *this = scale(*this, o, scl);
}
void Vec3f::scale(const Vec3f& o, const float scl) {
    *this = scale(*this, o, scl);
}

// VEC4
Vec4f::Vec4f() : x(0), y(0), z(0), w(0) {}
Vec4f::Vec4f(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
Vec3f Vec4f::toVec3f(bool norm) {
    if (norm) return Vec3f(x / w, y / w, z / w);
    return Vec3f(x, y, z);
}
Vec4f Vec4f::operator+(const Vec4f& v) {
    return Vec4f(x + v.x, y + v.y, z + v.z, w + v.w);
}
Vec4f Vec4f::operator+(const float t) {
    return Vec4f(x + t, y + t, z + t, w + t);
}
Vec4f Vec4f::operator-(const Vec4f& v) {
    return Vec4f(x - v.x, y - v.y, z - v.z, w - v.w);
}
Vec4f Vec4f::operator-(const float t) {
    return Vec4f(x - t, y - t, z - t, w - t);
}
Vec4f Vec4f::operator*(const float scl) {
    return Vec4f(x * scl, y * scl, z * scl, w * scl);
}
Vec4f Vec4f::operator/(const float scl) {
    return Vec4f(x / scl, y / scl, z / scl, w / scl);
}
void Vec4f::limit(float min, float max) {
    x = std::max(min, std::min(x, max));
    y = std::max(min, std::min(y, max));
    z = std::max(min, std::min(z, max));
    w = std::max(min, std::min(w, max));
}

// SoA structure Vecs

void Vec1f_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(float));
}
void Vec1f_ptr::free() {
    this->size = 0;
    cudaFree(x);
}
void Vec1f_ptr::operator+=(Vec1f_ptr& vec) {
    Vec1f_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}
void Vec1f_ptr::setAll(float val) {
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(x, val, size);
}

void Vec2f_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(float));
    cudaMalloc(&y, size * sizeof(float));
}
void Vec2f_ptr::free() {
    this->size = 0;
    cudaFree(x);
    cudaFree(y);
}
void Vec2f_ptr::operator+=(Vec2f_ptr& vec) {
    Vec2f_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y, y, size * sizeof(float), cudaMemcpyDeviceToDevice);

    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y + size, vec.y, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}
void Vec2f_ptr::setAll(float val) {
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(x, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(y, val, size);
}

void Vec3f_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(float));
    cudaMalloc(&y, size * sizeof(float));
    cudaMalloc(&z, size * sizeof(float));
}
void Vec3f_ptr::free() {
    this->size = 0;
    cudaFree(x);
    cudaFree(y);
    cudaFree(z);
}
void Vec3f_ptr::operator+=(Vec3f_ptr& vec) {
    Vec3f_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y, y, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.z, z, size * sizeof(float), cudaMemcpyDeviceToDevice);

    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y + size, vec.y, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.z + size, vec.z, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}
void Vec3f_ptr::setAll(float val) {
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(x, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(y, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(z, val, size);
}

void Vec4f_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(float));
    cudaMalloc(&y, size * sizeof(float));
    cudaMalloc(&z, size * sizeof(float));
    cudaMalloc(&w, size * sizeof(float));
}
void Vec4f_ptr::free() {
    this->size = 0;
    cudaFree(x);
    cudaFree(y);
    cudaFree(z);
    cudaFree(w);
}
void Vec4f_ptr::operator+=(Vec4f_ptr& vec) {
    Vec4f_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y, y, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.z, z, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.w, w, size * sizeof(float), cudaMemcpyDeviceToDevice);

    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y + size, vec.y, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.z + size, vec.z, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.w + size, vec.w, vec.size * sizeof(float), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}
void Vec4f_ptr::setAll(float val) {
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(x, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(y, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(z, val, size);
    setFloatAllKernel<<<(size + 255) / 256, 256>>>(w, val, size);
}

void Vec1lli_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(LLInt));
}
void Vec1lli_ptr::free() {
    this->size = 0;
    cudaFree(x);
}
void Vec1lli_ptr::operator+=(Vec1lli_ptr& vec) {
    Vec1lli_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(LLInt), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(LLInt), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}

void Vec2i_ptr::malloc(ULLInt size) {
    this->size = size;
    cudaMalloc(&x, size * sizeof(int));
    cudaMalloc(&y, size * sizeof(int));
}
void Vec2i_ptr::free() {
    this->size = 0;
    cudaFree(x);
    cudaFree(y);
}
void Vec2i_ptr::operator+=(Vec2i_ptr& vec) {
    Vec2i_ptr newVec;
    newVec.malloc(size + vec.size);

    cudaMemcpy(newVec.x, x, size * sizeof(int), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y, y, size * sizeof(int), cudaMemcpyDeviceToDevice);

    cudaMemcpy(newVec.x + size, vec.x, vec.size * sizeof(int), cudaMemcpyDeviceToDevice);
    cudaMemcpy(newVec.y + size, vec.y, vec.size * sizeof(int), cudaMemcpyDeviceToDevice);

    free();
    vec.free();
    *this = newVec;
}


// Atomics
__device__ bool atomicMinFloat(float* addr, float value) {
    int* addr_as_int = (int*)addr;
    int old = *addr_as_int, assumed;

    do {
        assumed = old;
        old = atomicCAS(addr_as_int, assumed, __float_as_int(fminf(value, __int_as_float(assumed))));
    } while (assumed != old);

    return __int_as_float(old) > value;
}

__device__ bool atomicMinDouble(double* addr, double value) {
    unsigned long long int* addr_as_ull = (unsigned long long int*)addr;
    unsigned long long int old = *addr_as_ull, assumed;

    do {
        assumed = old;
        old = atomicCAS(addr_as_ull, assumed, __double_as_longlong(fmin(value, __longlong_as_double(assumed))));
    } while (assumed != old);

    return __longlong_as_double(old) > value;
}

// Helper functions
__global__ void setFloatAllKernel(float *arr, float val, ULLInt size) {
    ULLInt i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size) arr[i] = val;
}
__global__ void setLLIntAllKernel(LLInt *arr, LLInt val, ULLInt size) {
    ULLInt i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size) arr[i] = val;
}