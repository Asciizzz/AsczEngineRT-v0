#ifndef MATFLAGS_CUH
#define MATFLAGS_CUH

struct MatFlags {
    // Color attributes
    bool has_kd, has_ka, has_ks, has_kr, has_ke;
    // Scalar attributes
    bool has_ns, has_ior, has_trnsprncy;
    // Texture attributes
    bool has_m_kd_txtr, has_m_ks_txtr, has_m_bump_txtr;
    bool has_m_nrml_txtr, has_m_alpha_txtr, has_m_kr_txtr;

    MatFlags();
};

#endif