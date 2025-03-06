#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczWin.cuh>
#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>

#include <RayTrace.cuh>
#include <PathTrace.cuh>
#include <RayCast.cuh>

__global__ void copyFrmBuffer(Flt3 *from, Flt3 *to, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) to[idx] = from[idx];
}

__global__ void addFrmBuffer(Flt3 *f1, Flt3 *f2, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) f1[idx] += f2[idx];
}

__global__ void divFrmBuffer(Flt3 *f1, Flt3 *f2, int size, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) f1[idx] = f2[idx] / count;
}

__global__ void bilateralFilter(Flt3* framebuffer, Flt3* output, int width, int height, int radius = 3, float sigma_spatial = 1.5f, float sigma_color = 0.2f) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    int x = tIdx % width;
    int y = tIdx / width;
    if (x >= width || y >= height) return;

    int idx = y * width + x;
    Flt3 centerColor = framebuffer[idx];

    Flt3 sumColor = {0, 0, 0};
    float sumWeight = 0.0f;

    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            int nIdx = ny * width + nx;
            Flt3 neighborColor = framebuffer[nIdx];

            // Spatial weight (Gaussian based on distance)
            float spatialWeight = expf(-(dx * dx + dy * dy) / (2.0f * sigma_spatial * sigma_spatial));

            // Color weight (Preserve edges by reducing weight for different colors)
            float colorDiff = (neighborColor - centerColor).mag();
            float colorWeight = expf(-(colorDiff * colorDiff) / (2.0f * sigma_color * sigma_color));

            float weight = spatialWeight * colorWeight;
            sumColor = sumColor + neighborColor * weight;
            sumWeight += weight;
        }
    }

    output[idx] = sumColor / sumWeight;
}

__global__ void initRandState(curandState *state, int width, int height, int seed) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < width * height) {
        curand_init(seed, idx, 0, &state[idx]);
    }
}

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin Win(1000, 1000, L"AsczEngineRT_v0");

    // =============== Initialize Important Managers ================

    // All managers
    AsczTxtr Txtr;
    AsczMat Mat;
    AsczMesh Mesh;
    AsczBvh Bvh;
    AsczCam Cam;

    // ====================== Some very scuffed init ==========================
    
    std::ifstream cfgFile(".cfg");
    std::string cfgLine;
    while (std::getline(cfgFile, cfgLine)) {
        if (cfgLine.size() == 0 || cfgLine[0] == '#') continue;

        std::stringstream ss(cfgLine);
        std::string type; ss >> type;

        if      (type == "CameraPos")  ss >> Cam.pos.x >> Cam.pos.y >> Cam.pos.z;
        else if (type == "CameraRot")  ss >> Cam.rot.x >> Cam.rot.y >> Cam.rot.z;
        else if (type == "CameraFov")  ss >> Cam.fov;
        else if (type == "VelSpec")    ss >> Cam.velSpec;
        else if (type == "SlowFactor") ss >> Cam.slowFactor;
        else if (type == "FastFactor") ss >> Cam.fastFactor;

        else if (type == "MaxDepth")   ss >> Bvh.MAX_DEPTH;
        else if (type == "BinCount")   ss >> Bvh.BIN_COUNT;
    };

    // ========================================================================
    // ======================= Some test geometries ===========================
    // ========================================================================

    // Test object loading
    // Load object file from .model
    std::ifstream objsFile(".model");
    std::string objLine;

    while (std::getline(objsFile, objLine)) {
        if (objLine.size() == 0 || objLine[0] == '#') continue;
        if (objLine[0] == '~') break;

        std::stringstream ss(objLine);

        std::string objPath;
        short objPlacement = 0;
        float objScale = 1.0f;

        ss >> objPath >> objPlacement >> objScale;

        Utils::appendObj(
            Mesh, Mat, Txtr,
            objPath.c_str(), objPlacement, objScale
        );
    }

    // ======================= Copy to device memory ==========================

    // Copy to device memory
    Txtr.toDevice();
    Mat.toDevice();
    Mesh.toDevice();

    Bvh.designBVH(Mesh);
    Bvh.toDevice();

    // ========================================================================
    // ========================================================================

    // Hide cursor
    ShowCursor(FALSE);

    short renderMode = 0;
    float falseAmbient = 0.1f; // Good for pitch black areas
    float curFalseAmb = falseAmbient;
    bool hasDebug = true;
    bool fakeShading = false;

    int accumulate = 0;

    Flt3 prevPos = Cam.pos;
    Flt3 prevRot = Cam.rot;
    float prevAptr = Cam.aperture;
    float prevFdst = Cam.focalDist;
    short prevMode = renderMode;

    curandState *d_randStates;
    cudaMalloc(&d_randStates, Win.width * Win.height * sizeof(curandState));
    initRandState<<<Win.blockCount, Win.threadCount>>>(d_randStates, Win.width, Win.height, accumulate);

    MSG msg = { 0 };
    while (msg.message != WM_QUIT) {
        FPS.startFrame();

        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // Press ESC to exit
        if (Win.keys[VK_ESCAPE]) break;

        // Press F1 to toggle focus
        if (Win.keys[VK_F1]) {
            Win.keys[VK_F1] = false;

            ShowCursor(Cam.focus);
            Cam.focus = !Cam.focus;

            if (Cam.focus) {
                POINT center = { Win.width / 2, Win.height / 2 };
                ClientToScreen(Win.hwnd, &center);
                SetCursorPos(center.x, center.y);
            }
        }

        // Press H to toggle debug
        if (Win.keys['H']) {
            Win.keys['H'] = false; hasDebug = !hasDebug;
        }

        // Press 1-3 to toggle render mode
        if (Win.keys['1']) {
            if (renderMode == 0)
                fakeShading = !fakeShading;

            Win.keys['1'] = false;
            renderMode = 0;
        }
        // else if (Win.keys['2']) {
        //     if (renderMode == 1) curFalseAmb = !curFalseAmb * falseAmbient;

        //     Win.keys['2'] = false; renderMode = 1;
        // }
        else if (Win.keys['2'] || Win.keys['3']) {
            Win.keys['2'] = false;
            Win.keys['3'] = false;
            renderMode = 2;
        }

        // Press F to change focal distance
        if (Win.keys['F']) {
            Win.keys['F'] = false;
            Cam.focalDist+= (Win.keys[VK_CONTROL] ? -1.0f : 1.0f) *
                            (Win.keys[VK_SHIFT] ? 1.0f : 0.1f);
        }
        // Press R to change aperture
        if (Win.keys['R']) {
            Win.keys['R'] = false;
            Cam.aperture += (Win.keys[VK_CONTROL] ? -1.0f : 1.0f) *
                            (Win.keys[VK_SHIFT] ? 0.1f : 0.005f);
        }

        if (Cam.focus) {
            // Get previous cursor position
            POINT prev;
            GetCursorPos(&prev);

            // Set cursor position to the center of the window
            POINT center = { Win.width / 2, Win.height / 2 };
            ClientToScreen(Win.hwnd, &center);
            SetCursorPos(center.x, center.y);

            float dx = prev.x - center.x;
            float dy = center.y - prev.y;

            // Update camera rotation
            Cam.rot.y -= dx * Cam.mSens * FPS.dTimeSec;
            Cam.rot.x += dy * Cam.mSens * FPS.dTimeSec;

            // CSGO perspective movement
            float vel = Cam.velSpec;
            bool k_w = Win.keys['W'];
            bool k_a = Win.keys['A'];
            bool k_s = Win.keys['S'];
            bool k_d = Win.keys['D'];
            bool k_ctrl = Win.keys[VK_CONTROL];
            bool k_shift = Win.keys[VK_SHIFT];

            // Hold ctrl to go slow, hold shift to go fast
            if (k_ctrl && !k_shift)      vel *= Cam.slowFactor;
            else if (k_shift && !k_ctrl) vel *= Cam.fastFactor;

            // Press W/S to move frwd/backward
            if (k_w && !k_s) Cam.pos += Cam.frwd * vel * FPS.dTimeSec;
            if (k_s && !k_w) Cam.pos -= Cam.frwd * vel * FPS.dTimeSec;

            // Press A/D to move left/rght
            if (k_a && !k_d) Cam.pos += Cam.rght * vel * FPS.dTimeSec;
            if (k_d && !k_a) Cam.pos -= Cam.rght * vel * FPS.dTimeSec;

            // Update camera
            Cam.update();

            center = { Win.width / 2, Win.height / 2 };
            ClientToScreen(Win.hwnd, &center);
            SetCursorPos(center.x, center.y);
        }

        // Render
        switch (renderMode) {
        case 0:
            raycastKernel<<<Win.blockCount, Win.threadCount>>>(
                Cam, Win.d_frmbuffer1, Win.width, Win.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_mi_x, Bvh.d_mi_y, Bvh.d_mi_z, Bvh.d_mx_x, Bvh.d_mx_y, Bvh.d_mx_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_gIdx,

                fakeShading
            );
            break;

        case 1:
            raytraceKernel<<<Win.blockCount, Win.threadCount>>>(
                Cam, Win.d_frmbuffer2, Win.width, Win.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lSrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_mi_x, Bvh.d_mi_y, Bvh.d_mi_z, Bvh.d_mx_x, Bvh.d_mx_y, Bvh.d_mx_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_gIdx,

                curFalseAmb
            );
            break;

        case 2:
            pathtraceKernel<<<Win.blockCount, Win.threadCount>>>(
                Cam, Win.d_frmbuffer1, Win.width, Win.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lSrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_mi_x, Bvh.d_mi_y, Bvh.d_mi_z, Bvh.d_mx_x, Bvh.d_mx_y, Bvh.d_mx_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_gIdx,

                d_randStates
            );

            bool changeRender = prevPos != Cam.pos ||
                                prevRot != Cam.rot ||
                                prevAptr != Cam.aperture ||
                                prevFdst != Cam.focalDist ||
                                prevMode != renderMode;
            if (changeRender) {
                accumulate = 1;
                copyFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer2, Win.width * Win.height);
            } else {
                accumulate ++;
                addFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer2, Win.d_frmbuffer1, Win.width * Win.height);
                divFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer2, Win.width * Win.height, accumulate);

                // Bilateral filter
                // bilateralFilter<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer3, Win.width, Win.height);
                copyFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer3, Win.width * Win.height);
            }
            prevPos = Cam.pos;
            prevRot = Cam.rot;
            prevAptr = Cam.aperture;
            prevFdst = Cam.focalDist;
            break;
        }
        prevMode = renderMode;

        if (hasDebug) {
            Win.appendDebug(L"AsczEngineRT_v0", Int3(155, 255, 155));
            Win.appendDebug(L"FPS: " + std::to_wstring(FPS.fps), Int3(0, 255, 0));
            Win.appendDebug(L"CAMERA", Int3(255, 0, 0));
            Win.appendDebug(L"Pos: " + std::to_wstring(Cam.pos.x) + L", " + std::to_wstring(Cam.pos.y) + L", " + std::to_wstring(Cam.pos.z), Int3(255));    
            Win.appendDebug(L"Rot: " + std::to_wstring(Cam.rot.x) + L", " + std::to_wstring(Cam.rot.y) + L", " + std::to_wstring(Cam.rot.z), Int3(255));
            Win.appendDebug(L"Fd: " + std::to_wstring(Cam.frwd.x) + L", " + std::to_wstring(Cam.frwd.y) + L", " + std::to_wstring(Cam.frwd.z), Int3(255));
            Win.appendDebug(L"Rg: " + std::to_wstring(Cam.rght.x) + L", " + std::to_wstring(Cam.rght.y) + L", " + std::to_wstring(Cam.rght.z), Int3(255));
            Win.appendDebug(L"Up: " + std::to_wstring(Cam.up.x) + L", " + std::to_wstring(Cam.up.y) + L", " + std::to_wstring(Cam.up.z), Int3(255));
            Win.appendDebug(L"Fov: " + std::to_wstring(Cam.fov * 180 / M_PI), Int3(255));
            Win.appendDebug(L"Aperature: " + std::to_wstring(Cam.aperture), Int3(255));
            Win.appendDebug(L"FocalDist: " + std::to_wstring(Cam.focalDist), Int3(255));

            // Retrieve the middle pixel color
            unsigned int color = Win.h_drawbuffer[Win.width * Win.height / 2 + Win.width / 2];
            int r = (color & 0x00FF0000) >> 16;
            int g = (color & 0x0000FF00) >> 8;
            int b = (color & 0x000000FF);
            Win.appendDebug(L"Color: " + std::to_wstring(r) + L", " + std::to_wstring(g) + L", " + std::to_wstring(b), Int3(255));
        }

        Win.Draw(renderMode + 1, hasDebug);

        FPS.endFrame();
    }

    // ========================================================================
    // ========================================================================

    // Free everything
    Txtr.freeDevice();
    Mat.freeDevice();
    Mesh.freeDevice();
    Bvh.freeDevice();
    Win.Terminate();

    return 0;
}
