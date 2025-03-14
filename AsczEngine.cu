#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczWin.cuh>
#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczCam.cuh>
#include <AsczFrame.cuh>

#include <RayCast.cuh>
#include <PathTraceSTD.cuh>
#include <PathTraceNEE.cuh>

#include <fstream>
#include <sstream>

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin Win(1600, 900, L"AsczEngineRT_v0");
    AsczFrame Frame(Win.width, Win.height);

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

    bool objStop = false;
    while (std::getline(objsFile, objLine)) {
        if (objLine.size() == 0 || objLine[0] == '#') continue;
        if (objLine[0] == '~') objStop = !objStop;
        if (objStop) continue;

        std::stringstream ss(objLine);

        std::string objPath;
        short   objPlacement = 0;
        float   objScl = 1.0f;
        float   objTx = 0.0f,
                objTy = 0.0f,
                objTz = 0.0f;
        float   objYaw = 0.0f;

        ss >> objPath >> objPlacement >> objScl >> objTx >> objTy >> objTz >> objYaw;

        // Convert to radians
        objYaw *= M_PI / 180.0f;

        Utils::appendObj(
            Mesh, Mat, Txtr,
            objPath.c_str(), objPlacement,
            objScl, objYaw, objTx, objTy, objTz
        );
    }

// ========================== PLAYGROUND ==================================

/*
    // Wave generation
    MeshStruct wave;
    float wave_start_x = -5.0f;
    float wave_start_z = -5.0f;
    float wave_start_y = 0.14f;
    float wave_height = 0.3f;
    float wave_move_x = 0.2f;
    float wave_move_z = 0.2f;
    int wave_step_x = 50;
    int wave_step_z = 50;

    for (float z = 0; z < wave_step_z; z++) {
        for (float x = 0; x < wave_step_x; x++) {
            float px = wave_start_x + x * wave_move_x;
            float pz = wave_start_z + z * wave_move_z;

            float py = sin(px) * cos(pz) * wave_height + wave_start_y;
            wave.v.push_back(Flt3(px, py, pz));
        }
    }

    AzMtl waveMat;
    waveMat.Rough = 0.05f;
    waveMat.Alb_r = 0.2;
    waveMat.Alb_g = 0.3;
    waveMat.Alb_b = 0.8;

    int wMat = Mat.append(waveMat, L"Wave", L"AsczEngine");

    // Append faces
    AABB waveAB;
    for (int z = 0; z < wave_step_z - 1; z++) {
        for (int x = 0; x < wave_step_x - 1; x++) {
            // Find: (x, z), (x+1, z), (x, z+1), (x+1, z+1)
            int v0idx = z * wave_step_x + x;
            int v1idx = z * wave_step_x + x + 1;
            int v2idx = (z + 1) * wave_step_x + x;
            int v3idx = (z + 1) * wave_step_x + x + 1;

            Flt3 v0 = wave.v[v0idx];
            Flt3 v1 = wave.v[v1idx];
            Flt3 v2 = wave.v[v2idx];
            Flt3 v3 = wave.v[v3idx];

            // Calculate normal by finding the gradient vector

            Flt3 n0 = {
                -cos(v0.x) * cos(v0.z) * wave_height,
                1,
                sin(v0.x) * sin(v0.z) * wave_height
            };
            n0 /= n0.x * n0.x + n0.y * n0.y + n0.z * n0.z;

            Flt3 n1 = {
                -cos(v1.x) * cos(v1.z) * wave_height,
                1,
                sin(v1.x) * sin(v1.z) * wave_height
            };
            n1 /= n1.x * n1.x + n1.y * n1.y + n1.z * n1.z;

            Flt3 n2 = {
                -cos(v2.x) * cos(v2.z) * wave_height,
                1,
                sin(v2.x) * sin(v2.z) * wave_height
            };
            n2 /= n2.x * n2.x + n2.y * n2.y + n2.z * n2.z;

            Flt3 n3 = {
                -cos(v3.x) * cos(v3.z) * wave_height,
                1,
                sin(v3.x) * sin(v3.z) * wave_height
            };
            n3 /= n3.x * n3.x + n3.y * n3.y + n3.z * n3.z;

            // Append normal
            wave.n.push_back(n0);
            wave.n.push_back(n1);
            wave.n.push_back(n2);
            wave.n.push_back(n3);

            int n0idx = wave.n.size() - 4;
            int n1idx = wave.n.size() - 3;
            int n2idx = wave.n.size() - 2;
            int n3idx = wave.n.size() - 1;

            // Expand AABB
            waveAB.expand(v0);
            waveAB.expand(v1);
            waveAB.expand(v2);
            waveAB.expand(v3);

            wave.fv.push_back(Int3(v0idx, v1idx, v2idx));
            wave.ft.push_back(-1);
            wave.fn.push_back(Int3(n0idx, n1idx, n2idx));
            wave.fm.push_back(wMat);

            wave.fv.push_back(Int3(v1idx, v3idx, v2idx));
            wave.ft.push_back(-1);
            wave.fn.push_back(Int3(n1idx, n3idx, n2idx));
            wave.fm.push_back(wMat);
        }
    }
    wave.O_AB = waveAB;

    // Append to mesh
    Mesh.append(wave);
*/

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

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_max_x, Bvh.d_max_y, Bvh.d_max_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

                altRender, // To add fake shading

                Frame.d_depth, Frame.d_mat
            );

            Frame.toDraw0(false, hasCrosshair);
        } 
        else if (renderMode == 1) {
            pathtraceSTDKernel<<<Frame.blockCount, Frame.blockSize>>>(
                Cam, Frame.d_fx0, Frame.d_fy0, Frame.d_fz0, Frame.width, Frame.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lsrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_max_x, Bvh.d_max_y, Bvh.d_max_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

                Frame.d_rand
            );

            if (altRender) {
                Frame.toDraw0(true, hasCrosshair);
            } else {
                Frame.add0();
                Frame.toDraw2(true);
            }
        }
        else if (renderMode == 2) {
            pathtraceNEEKernel<<<Frame.blockCount, Frame.blockSize>>>(
                Cam, Frame.d_fx0, Frame.d_fy0, Frame.d_fz0, Frame.width, Frame.height,

                Mesh.d_vx, Mesh.d_vy, Mesh.d_vz, Mesh.d_tx, Mesh.d_ty, Mesh.d_nx, Mesh.d_ny, Mesh.d_nz,
                Mesh.d_fv0, Mesh.d_fv1, Mesh.d_fv2, Mesh.d_ft0, Mesh.d_ft1, Mesh.d_ft2, Mesh.d_fn0, Mesh.d_fn1, Mesh.d_fn2, Mesh.d_fm,
                Mat.d_mtls, Mesh.d_lsrc, Mesh.lNum,
                Txtr.d_tr, Txtr.d_tg, Txtr.d_tb, Txtr.d_ta, Txtr.d_tw, Txtr.d_th, Txtr.d_toff,

                Bvh.d_min_x, Bvh.d_min_y, Bvh.d_min_z, Bvh.d_max_x, Bvh.d_max_y, Bvh.d_max_z, Bvh.d_pl, Bvh.d_pr, Bvh.d_lf, Bvh.d_fIdx,

                Frame.d_rand
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

            Win.appendDebug(L"CAMERA", 255, 100, 100);
            Win.appendDebug(L"Pos: " + std::to_wstring(Cam.px) + L", " + std::to_wstring(Cam.py) + L", " + std::to_wstring(Cam.pz), 255, 255, 255, 20);    
            Win.appendDebug(L"Pitch: " + std::to_wstring(Cam.rpit) + L"| Yaw: " + std::to_wstring(Cam.ryaw), 255, 255, 255, 20);
            Win.appendDebug(L"Fd: " + std::to_wstring(Cam.fw_x) + L", " + std::to_wstring(Cam.fw_y) + L", " + std::to_wstring(Cam.fw_z), 255, 255, 255, 20);
            Win.appendDebug(L"Rg: " + std::to_wstring(Cam.rg_x) + L", " + std::to_wstring(Cam.rg_y) + L", " + std::to_wstring(Cam.rg_z), 255, 255, 255, 20);
            Win.appendDebug(L"Up: " + std::to_wstring(Cam.up_x) + L", " + std::to_wstring(Cam.up_y) + L", " + std::to_wstring(Cam.up_z), 255, 255, 255, 20);
            Win.appendDebug(L"Fov: " + std::to_wstring(Cam.fov * 180 / M_PI), 255, 255, 255, 20);
            Win.appendDebug(L"Aperature: " + std::to_wstring(Cam.aperture), 255, 255, 255, 20);
            Win.appendDebug(L"FocalDist: " + std::to_wstring(Cam.focalDist), 255, 255, 255, 20);

            Win.appendDebug(L"Mesh", 100, 255, 100);
            Win.appendDebug(L"Vertices: " + std::to_wstring(Mesh.vNum), 255, 255, 255, 20);
            Win.appendDebug(L"Faces: " + std::to_wstring(Mesh.fNum), 255, 255, 255, 20);
            Win.appendDebug(L"Materials: " + std::to_wstring(Mat.mtlsNum), 255, 255, 255, 20);

            Win.appendDebug(L"Fragments", 100, 100, 255);

            // Retrieve the middle pixel color
            unsigned int color = Frame.h_draw[Win.width * Win.height / 2 + Win.width / 2];
            int r = (color & 0x00FF0000) >> 16;
            int g = (color & 0x0000FF00) >> 8;
            int b = (color & 0x000000FF);
            Win.appendDebug(L"Color: " + std::to_wstring(r) + L", " + std::to_wstring(g) + L", " + std::to_wstring(b), 255, 255, 255, 20);

            if (renderMode == 0) {
                int center = Frame.width * Frame.height / 2 + Frame.width / 2;

                float depth= -1.0f;
                int mat = -1;
                cudaMemcpy(&depth, Frame.d_depth + center, sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(&mat, Frame.d_mat + center, sizeof(int), cudaMemcpyDeviceToHost);

                if (mat > -1) {
                    std::wstring matName = Mat.names[mat];
                    std::wstring matPath = Mat.paths[mat];

                    Win.appendDebug(L"Depth: " + std::to_wstring(depth), 255, 255, 255, 20);
                    Win.appendDebug(L"Material:", 255, 255, 255, 20);
                    Win.appendDebug(L"Name: " + matName, 255, 255, 255, 40);
                    Win.appendDebug(L"Path: " + matPath, 255, 255, 255, 40);

                    const AzMtl &mtl = Mat.h_mtls[mat];
                    Win.appendDebug(L"Alb: " + std::to_wstring(mtl.Alb_r) + L", " + std::to_wstring(mtl.Alb_g) + L", " + std::to_wstring(mtl.Alb_b), 255, 255, 255, 40);

                    Win.appendDebug(L"Ems: " + std::to_wstring(mtl.Ems_r) + L", " + std::to_wstring(mtl.Ems_g) + L", " + std::to_wstring(mtl.Ems_b) + L", " + std::to_wstring(mtl.Ems_i), 255, 255, 255, 40);
                    Win.appendDebug(L"Rough: " + std::to_wstring(mtl.Rough), 255, 255, 255, 40);
                }
            }
        }

        Win.Draw(Frame.h_draw, hasDebug);

        FPS.endFrame();
    }

    // ========================================================================
    // ========================================================================

    // Free everything
    Win.Terminate();

    return 0;
}
