#include <MatAttrs.cuh>

MatAttrs::MatAttrs() {
    // Default material colors
    Ka = Vec3f(0.2f, 0.2f, 0.2f);
    Kd = Vec3f(0.8f, 0.8f, 0.8f);
    Ks = Vec3f(0.9f, 0.9f, 0.9f);
    Ke = Vec3f(0.0f, 0.0f, 0.0f);

    // Textures map IDs
    map_Ka = -1; map_Kd = -1;
    map_Ks = -1; map_Ke = -1;
    map_bump = -1;

    // Refraction and reflection
    Ni = 1.0f;
    reflectivity = 0.0f;
    Refract = Vec3f(0.0f, 0.0f, 0.0f);

    // Surface properties
    shiny = 20.0f;
    roughness = 0.05f;
    clearcoat = 0.5f;
    clearcoat_gloss = 0.2f;

    // Transparency
    Tr = 0.0f;
    transmit = Vec3f(0.0f, 0.0f, 0.0f);

    anisotropy = 0.5f;

    illum = 2;
}