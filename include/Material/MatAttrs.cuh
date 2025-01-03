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

struct MatAttrs {
    // Color attributes
    float kd_r, kd_g, kd_b;  // Diffuse color
    float ka_r, ka_g, ka_b;  // Ambient color
    float ks_r, ks_g, ks_b;  // Specular color
    float kr_r, kr_g, kr_b;  // Reflection color
    float ke_r, ke_g, ke_b;  // Emission color

    // Scalar attributes
    float ns;                // Shininess
    float ior;               // Index of refraction
    float trnsprncy;         // Transparency
    int illum_mdl;           // Illumination model

    // Texture indices (refer to the TextureManager)
    int map_kd_txtr_idx;     // Diffuse texture index
    int map_ks_txtr_idx;     // Specular texture index
    int map_bump_txtr_idx;   // Bump texture index
    int map_nrml_txtr_idx;   // Normal texture index
    int map_alpha_txtr_idx;  // Alpha texture index
    int map_kr_txtr_idx;     // Reflection texture index

    MatAttrs();
};

#endif