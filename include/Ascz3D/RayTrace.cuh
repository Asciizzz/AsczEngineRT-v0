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
    AsczCam camera, unsigned int *frmbuffer, int frmW, int frmH, // In-out
    // Primitive data
    float *vx, float *vy, float *vz, float *tx, float *ty, float *nx, float *ny, float *nz,
    // Materials
    AzMtl *mats,
    // Textures
    float *tr, float *tg, float *tb, float *ta, int *tw, int *th, int *toff,
    // Geometry data
    AzGeom *geom, int gNum,
    // Light data
    int *lSrc, int lNum, 
    // BVH data
    int *gIdx, DevNode *nodes, int nNum, 
    // Additional Debug Data
    bool falseAmbient = false
);

// __global__ void pathtraceKernel(
//     AsczCam camera, unsigned int *frmbuffer, int frmW, int frmH, // In-out
//     Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
//     AzMtl *mats, // Materials
//     Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
//     AzGeom *geom, int gNum, // Geometry data
//     int *lSrc, int lNum, // Light data
//     int *gIdx, DevNode *nodes, int nNum // BVH data
// );

#endif