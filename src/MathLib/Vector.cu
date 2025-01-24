#include <Matrix.cuh>

// VEC2f
Vec2f::Vec2f() : x(0), y(0) {}
Vec2f::Vec2f(float x, float y) : x(x), y(y) {}
float& Vec2f::operator[](int i) {
    if (i == 0) return x;
    return y;
}

Vec2f Vec2f::operator+(const Vec2f& v) const {
    return Vec2f(x + v.x, y + v.y);
}
Vec2f Vec2f::operator+(const float t) const {
    return Vec2f(x + t, y + t);
}
Vec2f Vec2f::operator-(const Vec2f& v) const {
    return Vec2f(x - v.x, y - v.y);
}
Vec2f Vec2f::operator-(const float t) const {
    return Vec2f(x - t, y - t);
}
Vec2f Vec2f::operator*(const float scl) const {
    return Vec2f(x * scl, y * scl);
}
Vec2f Vec2f::operator/(const float scl) const {
    return Vec2f(x / scl, y / scl);
}

// VEC3f
Vec3f::Vec3f() : x(0), y(0), z(0) {}
Vec3f::Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
Vec3f::Vec3f(float a) : x(a), y(a), z(a) {}
Vec4f Vec3f::toVec4f() {
    return Vec4f(x, y, z, 1);
}
float& Vec3f::operator[](int i) {
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
}

Vec3f Vec3f::operator-() const {
    return Vec3f(-x, -y, -z);
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
Vec3f Vec3f::operator&(const Vec3f& v) const { // Cross product
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
Vec3f Vec3f::bary(Vec3f v, Vec3f v0, Vec3f v1, Vec3f v2) {
    Vec3f v01 = v1 - v0, v02 = v2 - v0, v0v = v - v0;
    float d00 = v01 * v01, d01 = v01 * v02, d11 = v02 * v02, d20 = v0v * v01, d21 = v0v * v02;
    float d = d00 * d11 - d01 * d01;
    float a = (d11 * d20 - d01 * d21) / d;
    float b = (d00 * d21 - d01 * d20) / d;
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
float& Vec4f::operator[](int i) {
    if (i == 0) return x;
    if (i == 1) return y;
    if (i == 2) return z;
    return w;
}

Vec4f Vec4f::operator+(const Vec4f& v) const {
    return Vec4f(x + v.x, y + v.y, z + v.z, w + v.w);
}
Vec4f Vec4f::operator+(const float t) const {
    return Vec4f(x + t, y + t, z + t, w + t);
}
Vec4f Vec4f::operator-(const Vec4f& v) const {
    return Vec4f(x - v.x, y - v.y, z - v.z, w - v.w);
}
Vec4f Vec4f::operator-(const float t) const {
    return Vec4f(x - t, y - t, z - t, w - t);
}
Vec4f Vec4f::operator*(const float scl) const {
    return Vec4f(x * scl, y * scl, z * scl, w * scl);
}
Vec4f Vec4f::operator/(const float scl) const {
    return Vec4f(x / scl, y / scl, z / scl, w / scl);
}
void Vec4f::limit(float min, float max) {
    x = std::max(min, std::min(x, max));
    y = std::max(min, std::min(y, max));
    z = std::max(min, std::min(z, max));
    w = std::max(min, std::min(w, max));
}


// VEC3i
Vec3i::Vec3i() : x(0), y(0), z(0) {}
Vec3i::Vec3i(int x, int y, int z) : x(x), y(y), z(z) {}
Vec3i::Vec3i(int a) : x(a), y(a), z(a) {}

int& Vec3i::operator[](int i) {
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
}

Vec3i Vec3i::operator+(const Vec3i& v) const {
    return Vec3i(x + v.x, y + v.y, z + v.z);
}
Vec3i Vec3i::operator+(const int t) const {
    return Vec3i(x + t, y + t, z + t);
}
void Vec3i::operator+=(const Vec3i& v) {
    x += v.x; y += v.y; z += v.z;
}
void Vec3i::operator+=(const int t) {
    x += t; y += t; z += t;
}

Vec3i Vec3i::operator-(const Vec3i& v) const {
    return Vec3i(x - v.x, y - v.y, z - v.z);
}
Vec3i Vec3i::operator-(const int t) const {
    return Vec3i(x - t, y - t, z - t);
}
void Vec3i::operator-=(const Vec3i& v) {
    x -= v.x; y -= v.y; z -= v.z;
}
void Vec3i::operator-=(const int t) {
    x -= t; y -= t; z -= t;
}