#ifndef MATRTXFLAGS_H
#define MATRTXFLAGS_H

#include <MatAttrs.cuh>
#include <MatFlags.cuh>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

struct MatRtxFlags {
    // Ray tracing-specific flags
    bool reflection; // Material supports reflection
    bool refraction; // Material supports refraction
    bool shadow;     // Material interacts with shadows
    bool emission;   // Material emits light
    bool caustics;   // Material produces caustics
    bool fresnel;    // Material uses Fresnel reflections
    bool trnsprncy;  // Material is transparent
    bool specular;   // Material uses specular highlights

    MatRtxFlags();
};

#endif