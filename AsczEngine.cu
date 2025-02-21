#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczLight.cuh>
#include <AsczWin.cuh>
#include <AsczTxtr.cuh>
#include <AsczMtl.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>

#include <RayTrace.cuh>

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin WinMgr(1280, 720, L"AsczEngineRT");

    // =============== Initialize Important Managers ================

    // All managers
    AsczLight LightMgr;
    AsczTxtr TxtrMgr;
    AsczMtl MtlMgr;
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

        if (type == "LightSrc") {
            LightSrc lSrc; ss >>
                lSrc.pos.x >> lSrc.pos.y >> lSrc.pos.z >>
                lSrc.colr.x >> lSrc.colr.y >> lSrc.colr.z >>
                lSrc.intens >>
                lSrc.falloff >> lSrc.bias >> lSrc.exp >> lSrc.falloffDist;

            LightMgr.appendLight(lSrc);
        }

        if (type == "MaxDepth")
            ss >> BvhMgr.MAX_DEPTH;
        else if (type == "BinCount")
            ss >> BvhMgr.BIN_COUNT;
    };

    // ========================================================================
    // ========================= Buffer Allocation ============================
    // ========================================================================

    // Allocate frame buffers

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
            MeshMgr, MtlMgr, TxtrMgr,
            objPath.c_str(), objPlacement, objScale
        );
    }

    // ======================= Copy to device memory ==========================

    // Copy to device memory
    TxtrMgr.toDevice();
    MtlMgr.toDevice();
    MeshMgr.toDevice();

    BvhMgr.designBVH(MeshMgr);
    BvhMgr.toDevice();

    LightMgr.toDevice();

    // ========================================================================
    // ========================================================================

    // Hide cursor
    ShowCursor(FALSE);

    std::string LOG = "";

    MSG msg = { 0 };
    while (msg.message != WM_QUIT) {
        FPS.startFrame();

        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // Press ESC to exit
        if (WinMgr.keys[VK_ESCAPE]) break;

        if (CamMgr.focus) {
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

            // For the time being, press the arrow keys to look around
            bool k_up = WinMgr.keys[VK_UP];
            bool k_dw = WinMgr.keys[VK_DOWN];
            bool k_lf = WinMgr.keys[VK_LEFT];
            bool k_rt = WinMgr.keys[VK_RIGHT];

            if (k_up && !k_dw) CamMgr.rot.x += CamMgr.mSens * FPS.dTimeSec;
            if (k_dw && !k_up) CamMgr.rot.x -= CamMgr.mSens * FPS.dTimeSec;
            if (k_lf && !k_rt) CamMgr.rot.y -= CamMgr.mSens * FPS.dTimeSec;
            if (k_rt && !k_lf) CamMgr.rot.y += CamMgr.mSens * FPS.dTimeSec;

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
        } else {
            ClipCursor(nullptr);
        }

        // Render frmbuffer
        raytraceKernel<<<WinMgr.blockCount, WinMgr.threadCount>>>(
            CamMgr, WinMgr.d_framebuffer, WinMgr.width, WinMgr.height,
            TxtrMgr.d_txtrFlat, TxtrMgr.d_txtrPtr, MtlMgr.d_mtls,
            MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n, MeshMgr.d_geom, MeshMgr.gNum,
            BvhMgr.d_gIdx, BvhMgr.d_nodes, BvhMgr.nNum,
            LightMgr.d_lSrc, LightMgr.num
        );
        WinMgr.Draw();

        LOG = CamMgr.data();

        std::cout << "\r" << LOG << "    " << std::flush;

        FPS.endFrame();
    }

    // ========================================================================
    // ========================================================================

    // Free everything
    TxtrMgr.freeDevice();
    MtlMgr.freeDevice();
    MeshMgr.freeDevice();
    BvhMgr.freeDevice();

    WinMgr.Clear();

    return 0;
}
