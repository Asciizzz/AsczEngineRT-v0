#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>

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

__global__ void raytraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdx, DevNode *nodes, int nNum, // BVH data

    // Additional Debug Data
    bool falseAmbient = false
);

__global__ void pathtraceKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *lSrc, int lNum, // Light data
    int *gIdx, DevNode *nodes, int nNum // BVH data
);

__global__ void raycastKernel(
    AsczCam camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *tflat, TxPtr *tptr, // Textures
    AzMtl *mats, // Materials
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    AzGeom *geom, int gNum, // Geometry data
    int *gIdx, DevNode *nodes, int nNum // BVH data
);

#endif