#ifndef MATRTXFLAGS_H
#define MATRTXFLAGS_H

#include <MatAttrs.cuh>
#include <MatFlags.cuh>

struct MatRtxFlags {
    // Ray tracing-specific flags
    bool reflection;   // Material supports reflection
    bool refraction;   // Material supports refraction
    bool shadow;       // Material interacts with shadows
    bool emission;     // Material emits light
    bool caustics;     // Material produces caustics
    bool fresnel;      // Material uses Fresnel reflections
    bool trnsprncy;    // Material is transparent
    bool specular;     // Material uses specular highlights

    // Default constructor
    MatRtxFlags();
    // Manual initialization constructor
    MatRtxFlags(bool reflection, bool refraction, bool shadow, bool emission, bool caustics,
                bool fresnel, bool trnsprncy, bool specular);
    // Dynamic calculation constructor
    MatRtxFlags(const MatAttrs& attrs, const MatFlags& flags, int illum = 0);

private:
    void determineFlags(const MatAttrs& attrs, const MatFlags& flags, int illum);
};

#endif