#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <TxtrManager.cuh>
#include <MatManager.cuh>
#include <MeshManager.cuh>
#include <BvhManager.cuh>

__global__ void clearFrameBuffer(Flt3 *frmbuffer, int frmW, int frmH);

__global__ void iterativeRayTracing(
    Camera camera, Flt3 *frmbuffer, int frmW, int frmH, // In-out
    Flt4 *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Flt3 *mv, Flt2 *mt, Flt3 *mn, // Primitive data
    Int3 *mfv, Int3 *mft, Int3 *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH data
    int *fidx, DevNode *nodes, int nNum,

    Flt3 lightSrc
);

#endif