#ifndef AZSTRUCT_CUH
#define AZSTRUCT_CUH

#include <vector>
#include <fstream>
#include <iostream>

struct AzNode { // Flattened structure friendly for shader code
    float min_x= INFINITY, min_y= INFINITY, min_z= INFINITY;
    float max_x=-INFINITY, max_y=-INFINITY, max_z=-INFINITY;

    int cl = -1, cr = -1; // Children
    int ll = -1, lr = -1; // Primitive

    int depth = 0;
};


struct AzMesh {
    std::vector<float> vx, vy, vz;
    std::vector<float> nx, ny, nz;
    std::vector<float> tx, ty;

    std::vector<int> fv0, fv1, fv2; // Face vertices
    std::vector<int> fn0, fn1, fn2; // Face normals
    std::vector<int> ft0, ft1, ft2; // Face textures

    std::vector<int> fm; // Face materials
    std::vector<int> lsrc; // Light sources

    // Object AABB
    float O_AB_min_x = INFINITY, O_AB_min_y = INFINITY, O_AB_min_z = INFINITY;
    float O_AB_max_x = -INFINITY, O_AB_max_y = -INFINITY, O_AB_max_z = -INFINITY;

    void save(const std::string& filename) {
        std::ofstream outFile(filename, std::ios::binary);
        if (!outFile) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        // Lambda to write any vector
        auto write_vector = [&](auto& vec) {
            size_t size = vec.size();
            outFile.write(reinterpret_cast<const char*>(&size), sizeof(size));
            outFile.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(typename std::decay_t<decltype(vec)>::value_type));
        };

        // Save all vectors
        write_vector(vx); write_vector(vy); write_vector(vz);
        write_vector(nx); write_vector(ny); write_vector(nz);
        write_vector(tx); write_vector(ty);
        write_vector(fv0); write_vector(fv1); write_vector(fv2);
        write_vector(fn0); write_vector(fn1); write_vector(fn2);
        write_vector(ft0); write_vector(ft1); write_vector(ft2);
        write_vector(fm); write_vector(lsrc);

        // Save AABB bounds
        outFile.write(reinterpret_cast<const char*>(&O_AB_min_x), sizeof(O_AB_min_x));
        outFile.write(reinterpret_cast<const char*>(&O_AB_min_y), sizeof(O_AB_min_y));
        outFile.write(reinterpret_cast<const char*>(&O_AB_min_z), sizeof(O_AB_min_z));
        outFile.write(reinterpret_cast<const char*>(&O_AB_max_x), sizeof(O_AB_max_x));
        outFile.write(reinterpret_cast<const char*>(&O_AB_max_y), sizeof(O_AB_max_y));
        outFile.write(reinterpret_cast<const char*>(&O_AB_max_z), sizeof(O_AB_max_z));

        outFile.close();
    }

    void load(const std::string& filename) {
        std::ifstream inFile(filename, std::ios::binary);
        if (!inFile) {
            std::cerr << "Error opening file for reading: " << filename << std::endl;
            return;
        }

        // Lambda to read any vector
        auto read_vector = [&](auto& vec) {
            size_t size;
            inFile.read(reinterpret_cast<char*>(&size), sizeof(size));
            vec.resize(size);
            inFile.read(reinterpret_cast<char*>(vec.data()), size * sizeof(typename std::decay_t<decltype(vec)>::value_type));
        };

        // Load all vectors
        read_vector(vx); read_vector(vy); read_vector(vz);
        read_vector(nx); read_vector(ny); read_vector(nz);
        read_vector(tx); read_vector(ty);
        read_vector(fv0); read_vector(fv1); read_vector(fv2);
        read_vector(fn0); read_vector(fn1); read_vector(fn2);
        read_vector(ft0); read_vector(ft1); read_vector(ft2);
        read_vector(fm); read_vector(lsrc);

        // Load AABB bounds
        inFile.read(reinterpret_cast<char*>(&O_AB_min_x), sizeof(O_AB_min_x));
        inFile.read(reinterpret_cast<char*>(&O_AB_min_y), sizeof(O_AB_min_y));
        inFile.read(reinterpret_cast<char*>(&O_AB_min_z), sizeof(O_AB_min_z));
        inFile.read(reinterpret_cast<char*>(&O_AB_max_x), sizeof(O_AB_max_x));
        inFile.read(reinterpret_cast<char*>(&O_AB_max_y), sizeof(O_AB_max_y));
        inFile.read(reinterpret_cast<char*>(&O_AB_max_z), sizeof(O_AB_max_z));

        inFile.close();
    }
};

struct AzMtl {
    float Alb_r = 1.0f;
    float Alb_g = 1.0f;
    float Alb_b = 1.0f;
    int AlbMap = -1;

    float Rough = 1.0f;
    float Metal = 0.0f;
    // 0 < metal < 1 is physically impossible but AsczEngine allow it
    // for "dusty metal" or painted metal surfaces

    float Tr = 0.0f;
    float Ior = 1.0f;

    // Emission color and intensity
    float Ems_r = 0.0f;
    float Ems_g = 0.0f;
    float Ems_b = 0.0f;
    float Ems_i = 0.0f;

    // Debug values
    bool NoShade = false;
};

#endif