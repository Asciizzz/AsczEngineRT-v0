#ifndef MATERIAL_CUH
#define MATERIAL_CUH

// Just a placeholder class, will be removed
// once a more sophisticated material class
// is implemented

struct Material {
    float reflect = 0.0f;
    float transmit = 0.0f;

    float Fresnel = 0.0f;
    float Ni = 1.0f;

    int txtrIdx = -1;

    // Some special attributes
    bool isSky = false;
};

#endif