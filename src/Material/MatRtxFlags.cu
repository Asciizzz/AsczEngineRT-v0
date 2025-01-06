#include <MatRtxFlags.cuh>

MatRtxFlags::MatRtxFlags() :
    reflection(false), refraction(false), shadow(true),
    emission(false), caustics(false), fresnel(false),
    trnsprncy(false), specular(false)
{}