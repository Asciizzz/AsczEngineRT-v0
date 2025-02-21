#include <FpsHandler.cuh>
#include <Utility.cuh>

#include <AsczTxtr.cuh>
#include <AsczMtl.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczLight.cuh>
#include <AsczWin.cuh>

#include <FXAA.cuh>

#include <RayTrace.cuh>

int main() {
    // =================== Initialize FPS and Window ==============
    FpsHandler &FPS = FpsHandler::instance();
    AsczWin WinMgr(1280, 720, L"AsczEngineRT");

    // =================== Initialize window ===================

    // =============== Initialize Important Managers ================

    // All managers
    AsczTxtr TxtrMgr;
    AsczMtl MtlMgr;
    AsczMesh MeshMgr;
    AsczBvh BvhMgr;
    AsczLight LightMgr;

    // Create Camera
    // By logic, then this is CameraManager?
    // Idk, just a funny thought
    Camera CAMERA;

    // ====================== Some very scuffed init ==========================
    
    std::ifstream cfgFile(".cfg");
    std::string cfgLine;
    while (std::getline(cfgFile, cfgLine)) {
        if (cfgLine.size() == 0 || cfgLine[0] == '#') continue;

        std::stringstream ss(cfgLine);
        std::string type; ss >> type;

        if (type == "CameraPos")
            ss >> CAMERA.pos.x >> CAMERA.pos.y >> CAMERA.pos.z;
        else if (type == "CameraRot")
            ss >> CAMERA.rot.x >> CAMERA.rot.y >> CAMERA.rot.z;
        else if (type == "CameraFov")
            ss >> CAMERA.fov;
        else if (type == "VelSpec")
            ss >> CAMERA.velSpec;
        else if (type == "SlowFactor")
            ss >> CAMERA.slowFactor;
        else if (type == "FastFactor")
            ss >> CAMERA.fastFactor;

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

    WinMgr.Run();

    // ========================================================================
    // ========================================================================

    // Free device memory
    TxtrMgr.freeDevice();
    MtlMgr.freeDevice();
    MeshMgr.freeDevice();
    BvhMgr.freeDevice();

    return 0;
}
