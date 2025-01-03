#include <MatAttrs.cuh>

MatAttrs::MatAttrs() :
    kd_r(0.0f), kd_g(0.0f), kd_b(0.0f), // Diffuse color
    ka_r(0.0f), ka_g(0.0f), ka_b(0.0f), // Ambient color
    ks_r(0.0f), ks_g(0.0f), ks_b(0.0f), // Specular color
    kr_r(0.0f), kr_g(0.0f), kr_b(0.0f), // Reflection color
    ke_r(0.0f), ke_g(0.0f), ke_b(0.0f), // Emission color
    ns(0.0f), ior(1.0f), trnsprncy(0.0f), illum_mdl(0),
    map_kd_txtr_idx(-1), map_ks_txtr_idx(-1),
    map_bump_txtr_idx(-1), map_nrml_txtr_idx(-1),
    map_alpha_txtr_idx(-1), map_kr_txtr_idx(-1) {}