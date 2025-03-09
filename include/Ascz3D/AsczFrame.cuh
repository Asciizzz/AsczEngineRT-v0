#ifndef ASCZFRAME_CUH
#define ASCZFRAME_CUH

#include <curand_kernel.h>

class AsczFrame {
public:
    // Frame buffers
    /* Why 2 frame buffers?

    Some effects require the preservation of the previous frame.
    Examples: motion blur, antialiasing, bloom, etc.
    */
    float *d_fx0, *d_fy0, *d_fz0;
    float *d_fx1, *d_fy1, *d_fz1;

    // Accumulation Frame
    float *d_fx2, *d_fy2, *d_fz2;
    int f_acc = 0;

    // Random state
    curandState *d_rand;

    // Draw buffer
    unsigned int *d_draw;
    unsigned int *h_draw;

    // Other values
    float *d_depth;
    float *h_depth;

    int *d_mat;
    int *h_mat;

    int width;
    int height;
    int size;

    int blockSize = 256;
    int blockCount;

    AsczFrame(int w, int h);
    ~AsczFrame();

    void toDraw0(bool toneMap=true, bool crosshair=true);
    void toDraw1(bool toneMap=true, bool crosshair=true);
    void toDraw2(bool toneMap=true);

    void add0();
    void add1();
    void reset2();
};

#endif