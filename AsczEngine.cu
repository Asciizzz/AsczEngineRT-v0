#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczWin.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>
#include <AsczFrame.cuh>

#include <RayCast.cuh>
#include <PathTraceSTD.cuh>
#include <PathTraceNEE.cuh>

#include <fstream>
#include <sstream>
#include <iostream>

#define WAVE_HEIGHT 0.14f

float fXZ(float x, float z) {
    return sin(x) * cos(z) * WAVE_HEIGHT;
}
float fXZdx(float x, float z) {
    return -cos(x) * cos(z) * WAVE_HEIGHT;
}
float fXZdz(float x, float z) {
    return sin(x) * sin(z) * WAVE_HEIGHT;
}


int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin Win(1600, 900, L"AsczEngineRT_v0");
    AsczFrame Frame(Win.width, Win.height);

    // Paint the window black upon load
    unsigned int *black = new unsigned int[Win.width * Win.height];
    #pragma omp parallel
    for (int i = 0; i < Win.width * Win.height; i++) black[i] = 0xFF000000;
    
    Win.Draw(black);

    // =============== Initialize Important Managers ================

    AsczBvh Bvh;
    AsczCam Cam;

    // ====================== Some very scuffed init ==========================
    
    std::ifstream cfgFile(".cfg");
    std::string cfgLine;
    while (std::getline(cfgFile, cfgLine)) {
        if (cfgLine.size() == 0 || cfgLine[0] == '#') continue;

        std::stringstream ss(cfgLine);
        std::string type; ss >> type;

        if      (type == "CameraPos")  ss >> Cam.px >> Cam.py >> Cam.pz;
        else if (type == "CameraRot")  ss >> Cam.rpit >> Cam.ryaw;
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

    AzGlobal GLB;

    bool objStop = false;

    AzObj objs;
    bool createMany = false;
    while (std::getline(objsFile, objLine)) {
        if (objLine.size() == 0 || objLine[0] == '#') continue;
        if (objLine[0] == '~') objStop = !objStop;
        if (objStop) continue;

        std::stringstream ss(objLine);

        std::string objType;
        std::string objPath;
        short   objPlacement = 0;
        float   objScl = 1.0f;
        float   objTx = 0.0f,
                objTy = 0.0f,
                objTz = 0.0f;
        float   objYaw = 0.0f;

        ss >>
            objType >> objPath >> objPlacement >>
            objScl >> objTx >> objTy >> objTz >> objYaw;

        // Convert to radians
        objYaw *= M_PI / 180.0f;

        if (objType == "Create" || objType == "Instance") {
            AzObj obj = Utils::createAzb(
                objPath.c_str(), objPlacement, (objType == "Create"),
                objScl, objYaw, objTx, objTy, objTz
            );

            GLB.gulp(obj);
        }

        if (objType == "CreateMany") {
            createMany = true;
            objs = AzObj();
        }
        if (objType == "Add" && createMany) {
            AzObj obj = Utils::createAzb(
                objPath.c_str(), objPlacement, false,
                objScl, objYaw, objTx, objTy, objTz
            );
            objs.combine(obj);
        }
        if (objType == "Combine" && createMany) {
            createMany = false;

            GLB.gulp(objs);
            AzObj::save(objs, (objPath + "combined.azb").c_str());

            objs = AzObj();
        }


        if (objType == "Load") {
            AzObj obj = AzObj::load(objPath.c_str());
            GLB.gulp(obj);
        }
    }

    GLB.toDevice();
    GLB.computeAB();

    D_AzMesh &dMS = GLB.d_MS;
    D_AzMtl &dMT = GLB.d_MT;
    D_AzTxtr &dTX = GLB.d_TX;

// ========================== PLAYGROUND ==================================

/*
    // Wave generation
    AzObj wave;
    float wave_start_x = -100.0f;
    float wave_start_z = -100.0f;
    float wave_move_x = 0.1f;
    float wave_move_z = 0.1f;
    int wave_step_x = 2000;
    int wave_step_z = 2000;

    for (float z = 0; z < wave_step_z; z++) {
        for (float x = 0; x < wave_step_x; x++) {
            float px = wave_start_x + x * wave_move_x;
            float pz = wave_start_z + z * wave_move_z;

            float py = fXZ(px, pz);
            wave.MS.vx.push_back(px);
            wave.MS.vy.push_back(py);
            wave.MS.vz.push_back(pz);
        }
    }

    int waveMtl = wave.MT.push();
    wave.MT.Alb_r.back() = 0.2;
    wave.MT.Alb_g.back() = 0.3;
    wave.MT.Alb_b.back() = 0.6;
    wave.MT.Rough.back() = 0.5;

    // Append faces
    for (int z = 0; z < wave_step_z - 1; z++) {
        for (int x = 0; x < wave_step_x - 1; x++) {
            int v0idx = z * wave_step_x + x;
            int v1idx = z * wave_step_x + x + 1;
            int v2idx = (z + 1) * wave_step_x + x;
            int v3idx = (z + 1) * wave_step_x + x + 1;

            float v0x = wave.MS.vx[v0idx], v0y = wave.MS.vy[v0idx], v0z = wave.MS.vz[v0idx];
            float v1x = wave.MS.vx[v1idx], v1y = wave.MS.vy[v1idx], v1z = wave.MS.vz[v1idx];
            float v2x = wave.MS.vx[v2idx], v2y = wave.MS.vy[v2idx], v2z = wave.MS.vz[v2idx];
            float v3x = wave.MS.vx[v3idx], v3y = wave.MS.vy[v3idx], v3z = wave.MS.vz[v3idx];

            // Calculate normal by finding the gradient vector

            float n0x = fXZdx(v0x, v0z);
            float n0z = fXZdz(v0x, v0z);
            float n0_rmag = rsqrt(n0x * n0x + n0z * n0z + 1);

            int n0idx = wave.MS.nx.size();
            wave.MS.nx.push_back(n0x * n0_rmag);
            wave.MS.ny.push_back(n0_rmag);
            wave.MS.nz.push_back(n0z * n0_rmag);

            float n1x = fXZdx(v1x, v1z);
            float n1z = fXZdz(v1x, v1z);
            float n1_rmag = rsqrt(n1x * n1x + n1z * n1z + 1);

            int n1idx = wave.MS.nx.size();
            wave.MS.nx.push_back(n1x * n1_rmag);
            wave.MS.ny.push_back(n1_rmag);
            wave.MS.nz.push_back(n1z * n1_rmag);

            float n2x = fXZdx(v2x, v2z);
            float n2z = fXZdz(v2x, v2z);
            float n2_rmag = rsqrt(n2x * n2x + n2z * n2z + 1);

            int n2idx = wave.MS.nx.size();
            wave.MS.nx.push_back(n2x * n2_rmag);
            wave.MS.ny.push_back(n2_rmag);
            wave.MS.nz.push_back(n2z * n2_rmag);

            float n3x = fXZdx(v3x, v3z);
            float n3z = fXZdz(v3x, v3z);
            float n3_rmag = rsqrt(n3x * n3x + n3z * n3z + 1);

            int n3idx = wave.MS.nx.size();
            wave.MS.nx.push_back(n3x * n3_rmag);
            wave.MS.ny.push_back(n3_rmag);
            wave.MS.nz.push_back(n3z * n3_rmag);

            // Expand AABB
            wave.AB_x = fminf(wave.AB_x, fminf(v0x, fminf(v1x, fminf(v2x, v3x))));
            wave.AB_y = fminf(wave.AB_y, fminf(v0y, fminf(v1y, fminf(v2y, v3y))));
            wave.AB_z = fminf(wave.AB_z, fminf(v0z, fminf(v1z, fminf(v2z, v3z))));
            wave.AB_X = fmaxf(wave.AB_X, fmaxf(v0x, fmaxf(v1x, fmaxf(v2x, v3x))));
            wave.AB_Y = fmaxf(wave.AB_Y, fmaxf(v0y, fmaxf(v1y, fmaxf(v2y, v3y))));
            wave.AB_Z = fmaxf(wave.AB_Z, fmaxf(v0z, fmaxf(v1z, fmaxf(v2z, v3z))));


            wave.MS.fv0.push_back(v0idx); wave.MS.fv0.push_back(v1idx);
            wave.MS.fv1.push_back(v1idx); wave.MS.fv1.push_back(v3idx);
            wave.MS.fv2.push_back(v2idx); wave.MS.fv2.push_back(v2idx);

            wave.MS.ft0.push_back(-1); wave.MS.ft0.push_back(-1);
            wave.MS.ft1.push_back(-1); wave.MS.ft1.push_back(-1);
            wave.MS.ft2.push_back(-1); wave.MS.ft2.push_back(-1);

            wave.MS.fn0.push_back(n0idx); wave.MS.fn0.push_back(n1idx);
            wave.MS.fn1.push_back(n1idx); wave.MS.fn1.push_back(n3idx);
            wave.MS.fn2.push_back(n2idx); wave.MS.fn2.push_back(n2idx);

            wave.MS.fm.push_back(waveMtl); wave.MS.fm.push_back(waveMtl);
        }
    }
    wave.updateNumbers();

    AzObj::save(wave, "wave.azb");

//*/

    // ======================= Copy to device memory ==========================


    std::cout << "BVH Construction ... ";
    auto start = std::chrono::high_resolution_clock::now();

    Bvh.designBVH(GLB);
    Bvh.toDevice();

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Done in " <<
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    << "ms\n";

    // ========================================================================
    // ========================================================================

    // Hide cursor
    ShowCursor(FALSE);

    short renderMode = 0;
    bool altRender = false;

    bool hasDebug = true;
    bool hasCrosshair = true;

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

        // Scroll to change FOV
        if (Win.scroll != 0) {
            Cam.fov += Win.scroll * 0.1f;
            Win.scroll = 0;
        }

        // Press 1-3 to toggle render mode
        if (Win.keys['1']) {
            altRender = (renderMode == 0) * !altRender;
            Win.keys['1'] = false; renderMode = 0;
            Frame.reset2(); // Reset accumulation
        }
        else if (Win.keys['2']) {
            altRender = (renderMode == 1) * !altRender;
            Win.keys['2'] = false; renderMode = 1;
            Frame.reset2(); // Reset accumulation
        }
        else if (Win.keys['3']) {
            altRender = (renderMode == 2) * !altRender;
            Win.keys['3'] = false; renderMode = 2;
            Frame.reset2(); // Reset accumulation
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

            float dx = center.x - prev.x;
            float dy = center.y - prev.y;

            // Update camera rotation
            float dMsens = Cam.mSens * FPS.dTimeSec;
            Cam.ryaw += dx * dMsens;
            Cam.rpit += dy * dMsens;

            // CSGO perspective movement
            float dVel = Cam.velSpec * FPS.dTimeSec;
            bool k_w = Win.keys['W'];
            bool k_a = Win.keys['A'];
            bool k_s = Win.keys['S'];
            bool k_d = Win.keys['D'];
            bool k_ctrl = Win.keys[VK_CONTROL];
            bool k_shift = Win.keys[VK_SHIFT];

            // Hold ctrl to go slow, hold shift to go fast
            if (k_ctrl && !k_shift)      dVel *= Cam.slowFactor;
            else if (k_shift && !k_ctrl) dVel *= Cam.fastFactor;

            short moveFrwd = (k_w && !k_s) - (k_s && !k_w);
            short moveSide = (k_a && !k_d) - (k_d && !k_a);

            Cam.px += (Cam.fw_x * moveFrwd + Cam.rg_x * moveSide) * dVel;
            Cam.py += (Cam.fw_y * moveFrwd + Cam.rg_y * moveSide) * dVel;
            Cam.pz += (Cam.fw_z * moveFrwd + Cam.rg_z * moveSide) * dVel;

            // Update camera
            Cam.update();

            center = { Win.width / 2, Win.height / 2 };
            ClientToScreen(Win.hwnd, &center);
            SetCursorPos(center.x, center.y);
        }

        // Render
        if (renderMode == 0) {
            raycastKernel<<<Frame.blockCount, Frame.blockSize>>>(
                Cam, Frame.d_fx0, Frame.d_fy0, Frame.d_fz0, Frame.width, Frame.height,

                dMS.vx, dMS.vy, dMS.vz, dMS.tx, dMS.ty, dMS.nx, dMS.ny, dMS.nz,
                dMS.fv0, dMS.fv1, dMS.fv2, dMS.ft0, dMS.ft1, dMS.ft2, dMS.fn0, dMS.fn1, dMS.fn2, dMS.fm,

                dMT.Alb_r, dMT.Alb_g, dMT.Alb_b, dMT.AlbMap,
                dTX.r, dTX.g, dTX.b, dTX.a, dTX.w, dTX.h, dTX.off,

                Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_max_x, Bvh.d_max_y, Bvh.d_max_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

                altRender, // To add fake shading

                Frame.d_depth, Frame.d_mat
            );

            Frame.toDraw0(false, hasCrosshair);
        } 
        // else if (renderMode == 1) {
        //     pathtraceSTDKernel<<<Frame.blockCount, Frame.blockSize>>>(
        //         Cam, Frame.d_fx0, Frame.d_fy0, Frame.d_fz0, Frame.width, Frame.height,

        //         Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
        //         Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
        //         Mat.d_mtls, Mesh.d_lsrc, Mesh.lNum,
        //         Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

        //         Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_x, Bvh.d_y, Bvh.d_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

        //         Frame.d_rand
        //     );

        //     if (altRender) {
        //         Frame.toDraw0(true, hasCrosshair);
        //     } else {
        //         Frame.add0();
        //         Frame.toDraw2(true);
        //     }
        // }
        else if (renderMode == 2) {
            pathtraceNEEKernel<<<Frame.blockCount, Frame.blockSize>>>(
                Cam, Frame.d_fx0, Frame.d_fy0, Frame.d_fz0, Frame.width, Frame.height,

                dMS.vx, dMS.vy, dMS.vz, dMS.tx, dMS.ty, dMS.nx, dMS.ny, dMS.nz,
                dMS.fv0, dMS.fv1, dMS.fv2, dMS.ft0, dMS.ft1, dMS.ft2, dMS.fn0, dMS.fn1, dMS.fn2, dMS.fm,
                dMS.lsrc, dMS.l_num,

                dMT.Alb_r, dMT.Alb_g, dMT.Alb_b, dMT.AlbMap,
                dMT.Rough, dMT.Metal, dMT.Tr, dMT.Ior,
                dMT.Ems_r, dMT.Ems_g, dMT.Ems_b, dMT.Ems_i,

                dTX.r, dTX.g, dTX.b, dTX.a, dTX.w, dTX.h, dTX.off,

                Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_max_x, Bvh.d_max_y, Bvh.d_max_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

                // Frame.f_acc
                rand() * 1000
                // Frame.d_rand
            );

            if (altRender) {
                Frame.biliFilter0();
                Frame.toDraw0(true, hasCrosshair);
            } else {
                Frame.add0();
                Frame.toDraw2(true);
            }
        }

        if (hasDebug) {
            Win.appendDebug(L"AsczEngineRT_v0", 155, 255, 155);
            Win.appendDebug(L"FPS: " + std::to_wstring(FPS.fps), 0, 255, 0);

            std::wstring rAlt = altRender ? L"" : L" + Accumulation";
            std::wstring rMod = renderMode == 0 ? L"Raycast" :
                                renderMode == 1 ? L"PathTrace Std" + rAlt :
                                renderMode == 2 ? L"PathTrace NEE" + rAlt :
                                L"Unknown";
            Win.appendDebug(L"RENDER MODE: ", 0, 255, 255);
            Win.appendDebug(rMod, 255, 255, 255, 20);

            Win.appendDebug(L"WORLD", 100, 255, 100);
            Win.appendDebug(L"Vertex: " + std::to_wstring(GLB.MS.v_num), 255, 255, 255, 20);
            Win.appendDebug(L"Faces: " + std::to_wstring(GLB.MS.f_num), 255, 255, 255, 20);
            Win.appendDebug(L"Mtls: " + std::to_wstring(GLB.MT.num), 255, 255, 255, 20);
            Win.appendDebug(L"Textures: " + std::to_wstring(GLB.TX.num), 255, 255, 255, 20);

            Win.appendDebug(L"CAMERA", 255, 100, 100);
            Win.appendDebug(L"Pos: " + std::to_wstring(Cam.px) + L", " + std::to_wstring(Cam.py) + L", " + std::to_wstring(Cam.pz), 255, 255, 255, 20);    
            Win.appendDebug(L"Pitch: " + std::to_wstring(Cam.rpit) + L"| Yaw: " + std::to_wstring(Cam.ryaw), 255, 255, 255, 20);
            Win.appendDebug(L"Fd: " + std::to_wstring(Cam.fw_x) + L", " + std::to_wstring(Cam.fw_y) + L", " + std::to_wstring(Cam.fw_z), 255, 255, 255, 20);
            Win.appendDebug(L"Rg: " + std::to_wstring(Cam.rg_x) + L", " + std::to_wstring(Cam.rg_y) + L", " + std::to_wstring(Cam.rg_z), 255, 255, 255, 20);
            Win.appendDebug(L"Up: " + std::to_wstring(Cam.up_x) + L", " + std::to_wstring(Cam.up_y) + L", " + std::to_wstring(Cam.up_z), 255, 255, 255, 20);
            Win.appendDebug(L"Fov: " + std::to_wstring(Cam.fov * 180 / M_PI), 255, 255, 255, 20);
            Win.appendDebug(L"Aperature: " + std::to_wstring(Cam.aperture), 255, 255, 255, 20);
            Win.appendDebug(L"FocalDist: " + std::to_wstring(Cam.focalDist), 255, 255, 255, 20);
        }

        Win.Draw(Frame.h_draw, hasDebug);

        FPS.endFrame();
    }

    return 0;
}
