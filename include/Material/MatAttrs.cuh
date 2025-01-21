#ifndef MATATTRS_CUH
#define MATATTRS_CUH

/* Note Illum Model:

1. Color on and Ambient on
2. Highlight on
3. Reflection on and Ray trace on
4. Transparency: Glass on, Reflection: Ray trace on
5. Reflection: Fresnel on and Ray trace on
6. Transparency: Refraction on, Reflection: Fresnel off and Ray trace on
7. Transparency: Refraction on, Reflection: Fresnel on and Ray trace on
8. Reflection on and Ray trace off
9. Transparency: Glass on, Reflection: Ray trace off
10. Casts shadows onto invisible surfaces

*/
#include <Vector.cuh>

struct MatAttrs {
    float Ka[3];  // Color as RGB (0.0 - 1.0)
    float Kd[3];  // Color as RGB (0.0 - 1.0)
    float Ks[3];  // Color as RGB (0.0 - 1.0)
    float Ke[3];  // Color as RGB (0.0 - 1.0)

    int map_Ka;   // Ambient texture map
    int map_Kd;   // Diffuse texture map
    int map_Ks;   // Specular texture map
    int map_Ke;   // Emission texture map
    int map_bump; // Bump map

    float Ni;           // Index of refraction (typically between 1.0 and 2.0)
    float reflectivity; // Reflectivity factor (0.0 - 1.0)
    float Refract[3];   // Refraction color (RGB)

    float shiny;        // Shininess factor (higher values = sharper highlights)
    float roughness;    // Surface roughness (0.0 - 1.0)
    float clearcoat;    // Clearcoat intensity (0.0 - 1.0)
    float clearcoat_gloss; // Clearcoat glossiness (0.0 - 1.0)

    float Tr;           // Transparency (0.0 - 1.0)
    float transmit[3];  // Transmission color (RGB)

    float anisotropy;   // Anisotropy for simulating brushed metal-like surfaces

    int illum;          // Illumination model, typically 2 for diffuse+specular

    int map_displace;   // Displacement map
    int map_reflect;    // Reflection map

    _hst_dev_ MatAttrs();
};


#endif