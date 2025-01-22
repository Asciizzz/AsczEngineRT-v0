#ifndef RAYTRACE_CUH
#define RAYTRACE_CUH

#include <Camera.cuh>
#include <TxtrManager.cuh>

__global__ void clearFrameBuffer(Vec3f *framebuffer, int frmW, int frmH);

__global__ void iterativeRayTracing(
    Camera camera, Vec3f *framebuffer,
    Geom *geoms, int geomNum, int frmW, int frmH,
    Vec3f *txtrFlat, TxtrPtr *txtrPtr, int txtrCount
);

#endif