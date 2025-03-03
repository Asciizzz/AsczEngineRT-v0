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

__global__ void copyFrmBuffer(Flt3 *from, Flt3 *to, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) to[idx] = from[idx];
}

__global__ void resetFrmBuffer(Flt3 *frmbuffer, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) frmbuffer[idx] = Flt3(0.0f);
}

__global__ void addFrmBuffer(Flt3 *f1, Flt3 *f2, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) f1[idx] += f2[idx];
}

__global__ void divFrmBuffer(Flt3 *f1, Flt3 *f2, int size, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) f1[idx] = f2[idx] / count;
}

__global__ void temporalAA(Flt3 *f1, Flt3 *f2, int size, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) f1[idx] = (f1[idx] * count + f2[idx]) / (count + 1);
}

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin Win(1600, 900, L"AsczEngineRT_v0");

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

    bool pathTracing = false;
    float falseAmbient = 0.01f; // Good for pitch black areas
    float currentFalseAmbient = falseAmbient;
    bool hasDebug = true;

    int accumulate = 0;

    Flt3 prevPos = Cam.pos;
    Flt3 prevRot = Cam.rot;
    bool prevMode = pathTracing;

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

            Cam.focus = !Cam.focus;
            ShowCursor(Cam.focus);
        }

        // Press H to toggle debug
        if (Win.keys['H']) {
            Win.keys['H'] = false;
            hasDebug = !hasDebug;
        }

        // Press E to toggle false ambient
        if (Win.keys['E']) {
            Win.keys['E'] = false;
            currentFalseAmbient = currentFalseAmbient == 0.0f ? falseAmbient : 0.0f;
        }

        // Press Q to toggle path tracing
        if (Win.keys['Q']) {
            Win.keys['Q'] = false;
            pathTracing = !pathTracing;
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
            Cam.rot.y += dx * Cam.mSens * FPS.dTimeSec;
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

            // Press W/S to move forward/backward
            if (k_w && !k_s) Cam.pos += Cam.forward * vel * FPS.dTimeSec;
            if (k_s && !k_w) Cam.pos -= Cam.forward * vel * FPS.dTimeSec;

            // Press A/D to move left/right
            if (k_a && !k_d) Cam.pos -= Cam.right * vel * FPS.dTimeSec;
            if (k_d && !k_a) Cam.pos += Cam.right * vel * FPS.dTimeSec;

            // Update camera
            Cam.update();
        }

        if (Cam.focus) {
            POINT center = { Win.width / 2, Win.height / 2 };
            ClientToScreen(Win.hwnd, &center);
            SetCursorPos(center.x, center.y);
        }

        // Render
        if (!pathTracing)
            raytraceKernel<<<Win.blockCount, Win.threadCount>>>(
                Cam, Win.d_frmbuffer1, Win.width, Win.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lSrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_mi_x, Bvh.d_mi_y, Bvh.d_mi_z, Bvh.d_mx_x, Bvh.d_mx_y, Bvh.d_mx_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_gIdx,

                currentFalseAmbient
            );
        else
            pathtraceKernel<<<Win.blockCount, Win.threadCount>>>(
                Cam, Win.d_frmbuffer1, Win.width, Win.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lSrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_mi_x, Bvh.d_mi_y, Bvh.d_mi_z, Bvh.d_mx_x, Bvh.d_mx_y, Bvh.d_mx_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_gIdx,

                accumulate
            );

        if (prevPos != Cam.pos || prevRot != Cam.rot || prevMode != pathTracing) {
            accumulate = 1;

            copyFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer2, Win.width * Win.height);
        } else {
            accumulate++;
            addFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer2, Win.d_frmbuffer1, Win.width * Win.height);
            divFrmBuffer<<<Win.blockCount, Win.threadCount>>>(Win.d_frmbuffer1, Win.d_frmbuffer2, Win.width * Win.height, accumulate);
        }


        if (hasDebug) {
            Win.appendDebug(L"AsczEngineRT_v0", Int3(155, 255, 155));
            Win.appendDebug(L"FPS: " + std::to_wstring(FPS.fps), Int3(0, 255, 0));
            Win.appendDebug(L"CAMERA", Int3(255, 0, 0));
            Win.appendDebug(L"Pos: " + std::to_wstring(Cam.pos.x) + L", " + std::to_wstring(Cam.pos.y) + L", " + std::to_wstring(Cam.pos.z), Int3(255));    
            Win.appendDebug(L"Rot: " + std::to_wstring(Cam.rot.x) + L", " + std::to_wstring(Cam.rot.y) + L", " + std::to_wstring(Cam.rot.z), Int3(255));
            Win.appendDebug(L"Fd: " + std::to_wstring(Cam.forward.x) + L", " + std::to_wstring(Cam.forward.y) + L", " + std::to_wstring(Cam.forward.z), Int3(255));
            Win.appendDebug(L"Rg: " + std::to_wstring(Cam.right.x) + L", " + std::to_wstring(Cam.right.y) + L", " + std::to_wstring(Cam.right.z), Int3(255));
            Win.appendDebug(L"Up: " + std::to_wstring(Cam.up.x) + L", " + std::to_wstring(Cam.up.y) + L", " + std::to_wstring(Cam.up.z), Int3(255));
            Win.appendDebug(L"Fov: " + std::to_wstring(Cam.fov * 180 / M_PI), Int3(255));
        }

        Win.Draw(1, hasDebug);

        prevPos = Cam.pos;
        prevRot = Cam.rot;
        prevMode = pathTracing;

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
