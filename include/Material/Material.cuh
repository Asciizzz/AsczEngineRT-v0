#ifndef MATERIAL_CUH
#define MATERIAL_CUH

#include <MatRtxFlags.cuh>

struct Material {
    const char *name;
    MatAttrs attrs;
    MatFlags flags;
    MatRtxFlags rtxFlags;

    Material();
    Material(const char *name, const MatAttrs& attrs, const MatFlags& flags, const MatRtxFlags& rtxFlags);

    // Dynamic RtxFlags generation
    Material(const char *name, const MatAttrs& attrs, const MatFlags& flags);
    void determineRtxFlags(const MatAttrs& attrs, const MatFlags& flags);

    // Important: the following attributes are mostly pointers to the device memory
};

#endif