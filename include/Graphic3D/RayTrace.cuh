#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczLight.cuh>

/* Difference:

Real Time Ray Tracing:

- Only contain direct lighting (no Global Illumination)
- Few ray bounces
- Can run in real time

Static Path Tracing:

- Contain direct and indirect lighting (Global Illumination)
- Many ray bounces
- Only render a static frame

*/

struct Skybox {
    Flt3 *txtr;
    int w, h;
};

__global__ void realtimeRayTracing(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    Int3 *mfv, Int3 *mft, Int3 *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH data
    int *fidx, DevNode *nodes, int nNum,

    // Light data
    LightSrc *lSrc, int lNum,

    curandState *randState
);

#endif