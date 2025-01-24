#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <TxtrManager.cuh>
#include <MatManager.cuh>

__global__ void clearFrameBuffer(Vec3f *framebuffer, int frmW, int frmH);

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *framebuffer, int frmW, int frmH, // In-out
    Geom *geoms, int geomNum, // Will be replaced with BVH
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, // Textures
    Material *mats // Materials
);

#endif