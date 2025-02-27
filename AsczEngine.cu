#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczWin.cuh>
#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>

#include <RayTrace.cuh>

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin WinMgr(900, 900, L"AsczEngineRT_v0");

    // =============== Initialize Important Managers ================

    // All managers
    AsczTxtr TxtrMgr;
    AsczMat MatMgr;
    AsczMesh MeshMgr;
    AsczBvh BvhMgr;
    AsczCam CamMgr;

    // ====================== Some very scuffed init ==========================
    
    std::ifstream cfgFile(".cfg");
    std::string cfgLine;
    while (std::getline(cfgFile, cfgLine)) {
        if (cfgLine.size() == 0 || cfgLine[0] == '#') continue;

        std::stringstream ss(cfgLine);
        std::string type; ss >> type;

        if (type == "CameraPos")
            ss >> CamMgr.pos.x >> CamMgr.pos.y >> CamMgr.pos.z;
        else if (type == "CameraRot")
            ss >> CamMgr.rot.x >> CamMgr.rot.y >> CamMgr.rot.z;
        else if (type == "CameraFov")
            ss >> CamMgr.fov;
        else if (type == "VelSpec")
            ss >> CamMgr.velSpec;
        else if (type == "SlowFactor")
            ss >> CamMgr.slowFactor;
        else if (type == "FastFactor")
            ss >> CamMgr.fastFactor;

        if (type == "MaxDepth")
            ss >> BvhMgr.MAX_DEPTH;
        else if (type == "BinCount")
            ss >> BvhMgr.BIN_COUNT;
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
            MeshMgr, MatMgr, TxtrMgr,
            objPath.c_str(), objPlacement, objScale
        );
    }

    // ======================= Copy to device memory ==========================

    // Copy to device memory
    TxtrMgr.toDevice();
    MatMgr.toDevice();
    MeshMgr.toDevice();

    BvhMgr.designBVH(MeshMgr);
    BvhMgr.toDevice();

    // ========================================================================
    // ========================================================================

    // Hide cursor
    ShowCursor(FALSE);

    short rtMode = 0; // 0: Raytracing, 1: Pathtracing, 2: Raycasting
    bool falseAmbient = true; // Good for pitch black areas

    MSG msg = { 0 };
    while (msg.message != WM_QUIT) {
        FPS.startFrame();

        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        
        // Press ESC to exit
        if (WinMgr.keys[VK_ESCAPE]) break;

        // Press F1 to toggle focus
        if (WinMgr.keys[VK_F1]) {
            WinMgr.keys[VK_F1] = false;

            CamMgr.focus = !CamMgr.focus;
            ShowCursor(CamMgr.focus);
        }

        // Press E to toggle false ambient
        if (WinMgr.keys['E']) {
            WinMgr.keys['E'] = false;
            falseAmbient = !falseAmbient;
        }

        // Press 1 to toggle ray tracing
        if (WinMgr.keys['1']) {
            WinMgr.keys['1'] = false;
            rtMode = 0;
            CamMgr.focus = true;
        }
        // Press 2 to toggle ray casting
        if (WinMgr.keys['2']) {
            WinMgr.keys['2'] = false;
            rtMode = 1;
            CamMgr.focus = true;
        }
        // Press 3 to toggle path tracing
        if (WinMgr.keys['3'] && rtMode != 2) {
            WinMgr.keys['3'] = false;
            rtMode = 2;
            CamMgr.focus = false;

            // Render a single frame
            pathtraceKernel<<<WinMgr.blockCount, WinMgr.threadCount>>>(
                CamMgr, WinMgr.d_frmbuffer, WinMgr.width, WinMgr.height,
                TxtrMgr.d_flat, TxtrMgr.d_ptr, MatMgr.d_mtls,
                MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n, MeshMgr.d_geom, MeshMgr.gNum,
                MeshMgr.d_lSrc, MeshMgr.lNum,
                BvhMgr.d_gIdx, BvhMgr.d_nodes, BvhMgr.nNum
            );
        }

        if (CamMgr.focus && rtMode != 2) {
            // Get previous cursor position
            POINT prev;
            GetCursorPos(&prev);

            // Set cursor position to the center of the window
            POINT center = { WinMgr.width / 2, WinMgr.height / 2 };
            ClientToScreen(WinMgr.hwnd, &center);
            SetCursorPos(center.x, center.y);

            float dx = prev.x - center.x;
            float dy = center.y - prev.y;

            // Update camera rotation
            CamMgr.rot.y += dx * CamMgr.mSens * FPS.dTimeSec;
            CamMgr.rot.x += dy * CamMgr.mSens * FPS.dTimeSec;

            // CSGO perspective movement
            float vel = CamMgr.velSpec;
            bool k_w = WinMgr.keys['W'];
            bool k_a = WinMgr.keys['A'];
            bool k_s = WinMgr.keys['S'];
            bool k_d = WinMgr.keys['D'];
            bool k_ctrl = WinMgr.keys[VK_LCONTROL];
            bool k_shift = WinMgr.keys[VK_LSHIFT];

            // Hold ctrl to go slow, hold shift to go fast
            if (k_ctrl && !k_shift)      vel *= CamMgr.slowFactor;
            else if (k_shift && !k_ctrl) vel *= CamMgr.fastFactor;

            // Press W/S to move forward/backward
            if (k_w && !k_s) CamMgr.pos += CamMgr.forward * vel * FPS.dTimeSec;
            if (k_s && !k_w) CamMgr.pos -= CamMgr.forward * vel * FPS.dTimeSec;

            // Press A/D to move left/right
            if (k_a && !k_d) CamMgr.pos -= CamMgr.right * vel * FPS.dTimeSec;
            if (k_d && !k_a) CamMgr.pos += CamMgr.right * vel * FPS.dTimeSec;

            // Update camera
            CamMgr.update();
        }

        if (CamMgr.focus) {
            POINT center = { WinMgr.width / 2, WinMgr.height / 2 };
            ClientToScreen(WinMgr.hwnd, &center);
            SetCursorPos(center.x, center.y);
        }

        // Render
        if (rtMode == 0)
            raytraceKernel<<<WinMgr.blockCount, WinMgr.threadCount>>>(
                CamMgr, WinMgr.d_frmbuffer, WinMgr.width, WinMgr.height,
                TxtrMgr.d_flat, TxtrMgr.d_ptr, MatMgr.d_mtls,
                MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n, MeshMgr.d_geom, MeshMgr.gNum,
                MeshMgr.d_lSrc, MeshMgr.lNum,
                BvhMgr.d_gIdx, BvhMgr.d_nodes, BvhMgr.nNum,

                falseAmbient
            );
        else if (rtMode == 1)
            raycastKernel<<<WinMgr.blockCount, WinMgr.threadCount>>>(
                CamMgr, WinMgr.d_frmbuffer, WinMgr.width, WinMgr.height,
                TxtrMgr.d_flat, TxtrMgr.d_ptr, MatMgr.d_mtls,
                MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n, MeshMgr.d_geom, MeshMgr.gNum,
                BvhMgr.d_gIdx, BvhMgr.d_nodes, BvhMgr.nNum
            );

        WinMgr.appendDebug(L"AsczEngineRT_v0", Int3(155, 255, 155));
        std::wstring fps = rtMode == 2 ? L"-" : std::to_wstring(FPS.fps);
        WinMgr.appendDebug(L"FPS: " + fps, Int3(0, 255, 0));
        WinMgr.appendDebug(L"CAMERA", Int3(255, 0, 0));
        WinMgr.appendDebug(L"Pos: " + std::to_wstring(CamMgr.pos.x) + L", " + std::to_wstring(CamMgr.pos.y) + L", " + std::to_wstring(CamMgr.pos.z), Int3(255));    
        WinMgr.appendDebug(L"Rot: " + std::to_wstring(CamMgr.rot.x) + L", " + std::to_wstring(CamMgr.rot.y) + L", " + std::to_wstring(CamMgr.rot.z), Int3(255));
        WinMgr.appendDebug(L"Fd: " + std::to_wstring(CamMgr.forward.x) + L", " + std::to_wstring(CamMgr.forward.y) + L", " + std::to_wstring(CamMgr.forward.z), Int3(255));
        WinMgr.appendDebug(L"Rg: " + std::to_wstring(CamMgr.right.x) + L", " + std::to_wstring(CamMgr.right.y) + L", " + std::to_wstring(CamMgr.right.z), Int3(255));
        WinMgr.appendDebug(L"Up: " + std::to_wstring(CamMgr.up.x) + L", " + std::to_wstring(CamMgr.up.y) + L", " + std::to_wstring(CamMgr.up.z), Int3(255));
        WinMgr.appendDebug(L"Fov: " + std::to_wstring(CamMgr.fov * 180 / M_PI), Int3(255));

        WinMgr.Draw();

        FPS.endFrame();
    }

    // ========================================================================
    // ========================================================================

    // Free everything
    TxtrMgr.freeDevice();
    MatMgr.freeDevice();
    MeshMgr.freeDevice();
    BvhMgr.freeDevice();

    WinMgr.Terminate();

    return 0;
}
