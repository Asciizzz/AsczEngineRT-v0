#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <TxtrManager.cuh>
#include <MatManager.cuh>
#include <MeshManager.cuh>

__global__ void clearFrameBuffer(Vec3f *frmbuffer, int frmW, int frmH);

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *frmbuffer, int frmW, int frmH, // In-out
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats, // Materials
    // Mesh data
    Vec3f *mv, Vec2f *mt, Vec3f *mn, // Primitive data
    Vec3i *mfv, Vec3i *mft, Vec3i *mfn, int *mfm, // Face data
    int fNum, // Number of faces

    // BVH in the near future

    Vec3f lightSrc
);

#endif