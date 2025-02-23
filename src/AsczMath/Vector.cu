#include <Matrix.cuh>

// Flt2
Flt2::Flt2() : x(0), y(0) {}
Flt2::Flt2(float x, float y) : x(x), y(y) {}
float& Flt2::operator[](int i) {
    return i == 0 ? x : y;
}
float Flt2::operator[](int i) const {
    return i == 0 ? x : y;
}

Flt2 Flt2::operator+(const Flt2& v) const {
    return Flt2(x + v.x, y + v.y);
}
Flt2 Flt2::operator+(const float t) const {
    return Flt2(x + t, y + t);
}
Flt2 Flt2::operator-(const Flt2& v) const {
    return Flt2(x - v.x, y - v.y);
}
Flt2 Flt2::operator-(const float t) const {
    return Flt2(x - t, y - t);
}
Flt2 Flt2::operator*(const float scl) const {
    return Flt2(x * scl, y * scl);
}
Flt2 Flt2::operator/(const float scl) const {
    return Flt2(x / scl, y / scl);
}

// Flt3
Flt3::Flt3() : x(0), y(0), z(0) {}
Flt3::Flt3(float x, float y, float z) : x(x), y(y), z(z) {}
Flt3::Flt3(float a) : x(a), y(a), z(a) {}

float& Flt3::operator[](int i) {
    return i == 0 ? x : (i == 1 ? y : z);
}
float Flt3::operator[](int i) const {
    return i == 0 ? x : (i == 1 ? y : z);
}

Flt3 Flt3::operator-() const {
    return Flt3(-x, -y, -z);
}
Flt3 Flt3::operator+(const Flt3& v) const {
    return Flt3(x + v.x, y + v.y, z + v.z);
}
Flt3 Flt3::operator+(const float t) const {
    return Flt3(x + t, y + t, z + t);
}
Flt3 Flt3::operator-(const Flt3& v) const {
    return Flt3(x - v.x, y - v.y, z - v.z);
}
Flt3 Flt3::operator-(const float t) const {
    return Flt3(x - t, y - t, z - t);
}
Flt3 Flt3::operator*(const float scl) const {
    return Flt3(x * scl, y * scl, z * scl);
}
Flt3 Flt3::operator/(const float scl) const {
    return Flt3(x / scl, y / scl, z / scl);
}

Flt3 Flt3::operator+=(const Flt3& v) {
    x += v.x; y += v.y; z += v.z;
    return *this;
}
Flt3 Flt3::operator+=(const float t) {
    x += t; y += t; z += t;
    return *this;
}
Flt3 Flt3::operator-=(const Flt3& v) {
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}
Flt3 Flt3::operator-=(const float t) {
    x -= t; y -= t; z -= t;
    return *this;
}
Flt3 Flt3::operator*=(const float scl) {
    x *= scl; y *= scl; z *= scl;
    return *this;
}
Flt3 Flt3::operator/=(const float scl) {
    x /= scl; y /= scl; z /= scl;
    return *this;
}

Flt3 operator+(const float t, const Flt3& v) {
    return Flt3(v.x + t, v.y + t, v.z + t);
}
Flt3 operator-(const float t, const Flt3& v) {
    return Flt3(t - v.x, t - v.y, t - v.z);
}
Flt3 operator*(const float scl, const Flt3& v) {
    return Flt3(v.x * scl, v.y * scl, v.z * scl);
}
Flt3 operator/(const float scl, const Flt3& v) {
    return Flt3(scl / v.x, scl / v.y, scl / v.z);
}

// Comparison
bool Flt3::operator==(const Flt3& v) const {
    return x == v.x && y == v.y && z == v.z;
}
bool Flt3::operator!=(const Flt3& v) const {
    return x != v.x || y != v.y || z != v.z;
}

// Element-wise multiplication
Flt3 Flt3::operator&(const Flt3& v) const {
    return Flt3(x * v.x, y * v.y, z * v.z);
}
Flt3 Flt3::operator&=(const Flt3& v) {
    x *= v.x; y *= v.y; z *= v.z;
    return *this;
}
// Dot product
float Flt3::operator*(const Flt3& v) const {
    return x * v.x + y * v.y + z * v.z;
}
// Cross product
Flt3 Flt3::operator^(const Flt3& v) const {
    return Flt3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

// Some other common operations
float Flt3::mag() {
    return sqrt(x * x + y * y + z * z);
}
Flt3 Flt3::norm() {
    float m = mag();
    if (m == 0) return Flt3(0);
    return *this /= m;
}
void Flt3::abs() {
    x = x < 0 ? -x : x;
    y = y < 0 ? -y : y;
    z = z < 0 ? -z : z;
}

// Clamp
Flt3 Flt3::clamp(const Flt3& min, const Flt3& max) {
    x = x < min.x ? min.x : (x > max.x ? max.x : x);
    y = y < min.y ? min.y : (y > max.y ? max.y : y);
    z = z < min.z ? min.z : (z > max.z ? max.z : z);

    return *this;
}
Flt3 Flt3::clamp(const float min, const float max) {
    x = x < min ? min : (x > max ? max : x);
    y = y < min ? min : (y > max ? max : y);
    z = z < min ? min : (z > max ? max : z);

    return *this;
}


// Power
Flt3 Flt3::pow(float p) {
    x = powf(x, p);
    y = powf(y, p);
    z = powf(z, p);

    return *this;
}

// Other
bool Flt3::isZero() {
    return x == 0 && y == 0 && z == 0;
}   


// Transformations
Flt3 Flt3::translate(Flt3& v, const Flt3& t) {
    return v + t;
}

Flt3 Flt3::rotate(Flt3 &v, const Flt3 &o, const Flt3 &n, const float w) {
    // Note: the rodriques rotation is based on the right hand rule

    Flt3 dlt = v - o;
    Flt4 dlt4 = Flt4(dlt.x, dlt.y, dlt.z, 1);

    float cosW = cos(w), sinW = sin(w);

    // The Rodrigues formula
    Flt3 p = dlt*cosW + (n & v)*sinW + n*(n * v)*(1 - cosW);
    p += o;

    return p;
}

Flt3 Flt3::rotateX(Flt3 &v, const Flt3 &o, const float rx) {
    Flt3 dlt = v - o;
    Flt4 dlt4 = Flt4(dlt.x, dlt.y, dlt.z, 1);

    float cosX = cos(rx), sinX = sin(rx);
    float rX[4][4] = {
        {1, 0, 0, 0},
        {0, cosX, -sinX, 0},
        {0, sinX, cosX, 0},
        {0, 0, 0, 1}
    };

    Flt4 rVec4 = Mat4(rX) * dlt4;
    Flt3 rVec3 = rVec4.f3();
    rVec3 += o;

    return rVec3;
}
Flt3 Flt3::rotateY(Flt3 &v, const Flt3 &o, const float ry) {
    Flt3 dlt = v - o;
    Flt4 dlt4 = Flt4(dlt.x, dlt.y, dlt.z, 1);

    float cosY = cos(ry), sinY = sin(ry);
    float rY[4][4] = {
        {cosY, 0, sinY, 0},
        {0, 1, 0, 0},
        {-sinY, 0, cosY, 0},
        {0, 0, 0, 1}
    };

    Flt4 rVec4 = Mat4(rY) * dlt4;
    Flt3 rVec3 = rVec4.f3();
    rVec3 += o;

    return rVec3;
}
Flt3 Flt3::rotateZ(Flt3 &v, const Flt3 &o, const float rz) {
    Flt3 dlt = v - o;
    Flt4 dlt4 = Flt4(dlt.x, dlt.y, dlt.z, 1);

    float cosZ = cos(rz), sinZ = sin(rz);
    float rZ[4][4] = {
        {cosZ, -sinZ, 0, 0},
        {sinZ, cosZ, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    Flt4 rVec4 = Mat4(rZ) * dlt4;
    Flt3 rVec3 = rVec4.f3();
    rVec3 += o;

    return rVec3;
}

Flt3 Flt3::scale(Flt3& v, const Flt3& o, const Flt3& scl) {
    Flt3 dlt = v - o;
    return Flt3(
        o.x + dlt.x * scl.x,
        o.y + dlt.y * scl.y,
        o.z + dlt.z * scl.z
    );
}
Flt3 Flt3::scale(Flt3& v, const Flt3& o, const float scl) {
    return scale(v, o, Flt3(scl));
}

// Transformations but on self
void Flt3::translate(const Flt3& t) {
    *this += t;
}

void Flt3::rotateX(const Flt3& o, const float rx) {
    *this = rotateX(*this, o, rx);
}
void Flt3::rotateY(const Flt3& o, const float ry) {
    *this = rotateY(*this, o, ry);
}
void Flt3::rotateZ(const Flt3& o, const float rz) {
    *this = rotateZ(*this, o, rz);
}

void Flt3::scale(const Flt3& o, const Flt3& scl) {
    *this = scale(*this, o, scl);
}
void Flt3::scale(const Flt3& o, const float scl) {
    *this = scale(*this, o, scl);
}

// VEC4
Flt4::Flt4() : x(0), y(0), z(0), w(0) {}
Flt4::Flt4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
Flt3 Flt4::f3(bool norm) {
    return norm ? (Flt3(x, y, z) / w) : Flt3(x, y, z);
}
float& Flt4::operator[](int i) {
    return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
}
float Flt4::operator[](int i) const {
    return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
}

Flt4 Flt4::operator+(const Flt4& v) const {
    return Flt4(x + v.x, y + v.y, z + v.z, w + v.w);
}
Flt4 Flt4::operator+(const float t) const {
    return Flt4(x + t, y + t, z + t, w + t);
}
Flt4 Flt4::operator-(const Flt4& v) const {
    return Flt4(x - v.x, y - v.y, z - v.z, w - v.w);
}
Flt4 Flt4::operator-(const float t) const {
    return Flt4(x - t, y - t, z - t, w - t);
}
Flt4 Flt4::operator*(const float scl) const {
    return Flt4(x * scl, y * scl, z * scl, w * scl);
}
Flt4 Flt4::operator/(const float scl) const {
    return Flt4(x / scl, y / scl, z / scl, w / scl);
}


// Int3
Int3::Int3() : x(0), y(0), z(0) {}
Int3::Int3(int x, int y, int z) : x(x), y(y), z(z) {}
Int3::Int3(int a) : x(a), y(a), z(a) {}

int& Int3::operator[](int i) {
    return i == 0 ? x : (i == 1 ? y : z);
}

Int3 Int3::operator+(const Int3& v) const {
    return Int3(x + v.x, y + v.y, z + v.z);
}
Int3 Int3::operator+(const int t) const {
    return Int3(x + t, y + t, z + t);
}
void Int3::operator+=(const Int3& v) {
    x += v.x; y += v.y; z += v.z;
}
void Int3::operator+=(const int t) {
    x += t; y += t; z += t;
}

Int3 Int3::operator-(const Int3& v) const {
    return Int3(x - v.x, y - v.y, z - v.z);
}
Int3 Int3::operator-(const int t) const {
    return Int3(x - t, y - t, z - t);
}
void Int3::operator-=(const Int3& v) {
    x -= v.x; y -= v.y; z -= v.z;
}
void Int3::operator-=(const int t) {
    x -= t; y -= t; z -= t;
}