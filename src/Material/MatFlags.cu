#include <MatFlags.cuh>

MatFlags::MatFlags() :
    has_kd(false), has_ka(false), has_ks(false), has_kr(false), has_ke(false),
    has_ns(false), has_ior(false), has_trnsprncy(false),
    has_m_kd_txtr(false), has_m_ks_txtr(false), has_m_bump_txtr(false),
    has_m_nrml_txtr(false), has_m_alpha_txtr(false), has_m_kr_txtr(false)
{}