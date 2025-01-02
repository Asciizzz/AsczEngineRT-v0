#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Vector.cuh>

#define VectF std::vector<float>

#define VectI std::vector<int>
#define VectLLI std::vector<LLInt>
#define VectULLI std::vector<ULLInt>

#define VectBool std::vector<bool>

#define VectStr std::vector<std::string>

struct Triangle {
    Vec3f v0, v1, v2;
    // Vec2f t0, t1, t2; // Will be ignored for now
    Vec3f c0, c1, c2; // Placeholder, to test interpolation
    Vec3f n0, n1, n2;

    bool reflect = false; // Test reflection
    bool display = true; // For debugging

    // Some helper functions
    void uniformColor(Vec3f color) {
        c0 = color;
        c1 = color;
        c2 = color;
    }
    void uniformNormal(Vec3f normal) {
        n0 = normal;
        n1 = normal;
        n2 = normal;
    }
    void normAll() {
        n0.norm();
        n1.norm();
        n2.norm();
    }
};

class Utils {
public:
    static std::vector<Triangle> readObjFile(std::string name, std::string path, short fIdxBased=1, short placement=0);
};

#endif