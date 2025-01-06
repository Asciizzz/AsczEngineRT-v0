#include <MatAttrs.cuh>

MatAttrs::MatAttrs() {
    // Colors
    Ka[0] = 0.2f; Ka[1] = 0.2f; Ka[2] = 0.2f;
    Kd[0] = 0.8f; Kd[1] = 0.8f; Kd[2] = 0.8f;
    Ks[0] = 0.9f; Ks[1] = 0.9f; Ks[2] = 0.9f;
    Ke[0] = 0.0f; Ke[1] = 0.0f; Ke[2] = 0.0f;
    // Textures map IDs
    map_Ka = -1; map_Kd = -1;
    map_Ks = -1; map_Ke = -1;
    map_bump = -1;

    // Refraction and reflection
    Ni = 1.0f;
    reflectivity = 0.0f;
    Refract[0] = 0.0f; Refract[1] = 0.0f; Refract[2] = 0.0f;

    // Surface properties
    shiny = 20.0f;
    roughness = 0.05f;
    clearcoat = 0.5f;
    clearcoat_gloss = 0.2f;

    // Transparency
    Tr = 0.0f;
    transmit[0] = 0.5f; transmit[1] = 0.5f; transmit[2] = 0.5f;

    anisotropy = 0.5f;

    illum = 2;

    // Other maps
    map_displace = -1;
    map_reflect = -1;
}