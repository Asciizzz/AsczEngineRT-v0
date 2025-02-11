#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <AsczTxtr.cuh>
#include <AsczMtl.cuh>
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

_glb_ void raytraceKernel(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mtls, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *gIdx, DevNode *nodes, int nNum, // BVH data
    LightSrc *lSrc, int lNum, // Light data
    curandState *randState, // Random state

    // Some constants
    Ray *rstack, int *nstack,
    const int MAX_RAYS, const int MAX_NODES 
);

_glb_ void pathtraceKernel(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mtls, // Materials
    // Mesh data
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data

    // BVH data
    int *gIdx, DevNode *nodes, int nNum,

    // Light data
    LightSrc *lSrc, int lNum,

    curandState *randState
);

#endif