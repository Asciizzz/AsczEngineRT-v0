#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Utility.cuh>
#include <random>

#include <AsczTxtr.cuh>
#include <AsczMtl.cuh>
#include <AsczMesh.cuh>
#include <AsczBvh.cuh>
#include <AsczLight.cuh>

#include <FXAA.cuh>

#include <RayTrace.cuh>
#include <SFMLTexture.cuh>

#include <curand_kernel.h>

__global__ void initCurand(curandState *randState, int n, unsigned long long seed = 0) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;

    curand_init(seed, idx, 0, &randState[idx]);
}

int main() {
    // =================== Initialize FPS and Log ==============
    FpsHandler &FPS = FpsHandler::instance();
    CsLogHandler LOG = CsLogHandler();
    LOG.fontSize = 24;

    // =================== Initialize window ===================

    // Create window (set to fullscreen)
    int winW = sf::VideoMode::getDesktopMode().width;
    int winH = sf::VideoMode::getDesktopMode().height;

    sf::RenderWindow window(sf::VideoMode(winW, winH), "AsczEngine");
    sf::Mouse::setPosition(sf::Vector2i(winW / 2, winH / 2), window);
    window.setMouseCursorVisible(false);

    // Crosshair
    int crosshairSize = 10;
    int crosshairThick = 2;
    sf::Color crosshairColor = sf::Color::Green;
    sf::RectangleShape crosshair1(
        sf::Vector2f(crosshairSize + crosshairThick, crosshairThick)
    );
    crosshair1.setPosition(winW / 2 - crosshairSize / 2, winH / 2);
    crosshair1.setFillColor(crosshairColor);
    sf::RectangleShape crosshair2(
        sf::Vector2f(crosshairThick, crosshairSize + crosshairThick)
    );
    crosshair2.setPosition(winW / 2, winH / 2 - crosshairSize / 2);
    crosshair2.setFillColor(crosshairColor);

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

    // Set frame buffer properties
    float frmScl = 1;
    int frmW = winW / frmScl;
    int frmH = winH / frmScl;

    bool hasFXAA = true;

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

        if (type == "FrameScl") {
            ss >> frmScl;
            frmW = winW / frmScl;
            frmH = winH / frmScl;
        }

        if (type == "FXAA") {
            ss >> hasFXAA;
        }

        if (type == "MaxDepth")
            ss >> BvhMgr.MAX_DEPTH;
        else if (type == "BinCount")
            ss >> BvhMgr.BIN_COUNT;
    };

    // ========================================================================
    // ========================= Buffer Allocation ============================
    // ========================================================================
    
    // Allocate frame buffer
    Flt3 *d_frmbuffer1, *d_frmbuffer2;
    cudaMalloc(&d_frmbuffer1, frmW * frmH * sizeof(Flt3));
    cudaMalloc(&d_frmbuffer2, frmW * frmH * sizeof(Flt3));

    // Allocate luminance buffer
    float *d_luminance; bool *d_edge;
    cudaMalloc(&d_luminance, frmW * frmH * sizeof(float));
    cudaMalloc(&d_edge, frmW * frmH * sizeof(bool));

    // Allocate Curand states
    curandState *d_randState;
    int threads = 256;
    int blocks = (frmW * frmH + threads - 1) / threads;
    cudaMalloc(&d_randState, frmW * frmH * sizeof(curandState));
    initCurand<<<blocks, threads>>>(d_randState, frmW * frmH, time(0));
    cudaDeviceSynchronize();

    // Create SFML texture
    SFMLTexture SFTex(frmW, frmH);
    SFTex.sprite.setScale(frmScl, frmScl);

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

    // Main loop
    while (window.isOpen()) {
        // Frame start
        FPS.startFrame();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed ||
                sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {      
                window.close();
            }

            // Scroll to change the field of view
            if (event.type == sf::Event::MouseWheelScrolled) {
                if (event.mouseWheelScroll.delta > 0) CAMERA.fov += 0.1;
                else if (event.mouseWheelScroll.delta < 0) CAMERA.fov -= 0.1;

                if (CAMERA.fov < 0.1) CAMERA.fov += 0.1;
                if (CAMERA.fov > M_PI - 0.1) CAMERA.fov -= 0.1;
            }

            if (event.type == sf::Event::KeyPressed) {

            // Press f1 to toggle camera focus
                if (event.key.code == sf::Keyboard::F1) {
                    CAMERA.focus = !CAMERA.focus;
                    // To avoid sudden camera movement when changing focus
                    sf::Mouse::setPosition(
                        sf::Vector2i(winW / 2, winH / 2), window);

                    // Hide cursor
                    window.setMouseCursorVisible(!CAMERA.focus);
                }

            // Press F to toggle FXAA
                if (event.key.code == sf::Keyboard::F) {
                    hasFXAA = !hasFXAA;
                }
            }
        }

        // Setting input activities
        bool m_left = sf::Mouse::isButtonPressed(sf::Mouse::Left);
        bool m_right = sf::Mouse::isButtonPressed(sf::Mouse::Right);

        bool k_ctrl = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl);
        bool k_shift = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift);

        bool k_w = sf::Keyboard::isKeyPressed(sf::Keyboard::W);
        bool k_a = sf::Keyboard::isKeyPressed(sf::Keyboard::A);
        bool k_s = sf::Keyboard::isKeyPressed(sf::Keyboard::S);
        bool k_d = sf::Keyboard::isKeyPressed(sf::Keyboard::D);
        bool k_space = sf::Keyboard::isKeyPressed(sf::Keyboard::Space);

        bool k_q = sf::Keyboard::isKeyPressed(sf::Keyboard::Q);
        bool k_e = sf::Keyboard::isKeyPressed(sf::Keyboard::E);
        bool k_t = sf::Keyboard::isKeyPressed(sf::Keyboard::T);

        // Camera movement
        if (CAMERA.focus) {
        // Camera look around
            sf::Vector2i mousepos = sf::Mouse::getPosition(window);
            sf::Mouse::setPosition(sf::Vector2i(winW / 2, winH / 2), window);

            // Move from center
            int dMx = mousepos.x - winW / 2;
            int dMy = mousepos.y - winH / 2;

            // Camera look around
            CAMERA.rot.x -= dMy * CAMERA.mSens * FPS.dTimeSec;
            CAMERA.rot.y += dMx * CAMERA.mSens * FPS.dTimeSec;

        // CSGO perspective movement
            float vel = CAMERA.velSpec;
            // Hold ctrl to go slow, hold shift to go fast
            if (k_ctrl && !k_shift)      vel *= CAMERA.slowFactor;
            else if (k_shift && !k_ctrl) vel *= CAMERA.fastFactor;
            // Press W/S to move forward/backward
            if (k_w && !k_s) CAMERA.pos += CAMERA.forward * vel * FPS.dTimeSec;
            if (k_s && !k_w) CAMERA.pos -= CAMERA.forward * vel * FPS.dTimeSec;
            // Press A/D to move left/right
            if (k_a && !k_d) CAMERA.pos -= CAMERA.right * vel * FPS.dTimeSec;
            if (k_d && !k_a) CAMERA.pos += CAMERA.right * vel * FPS.dTimeSec;
        }
        // Update camera
        CAMERA.update();

        // Render frmbuffer
        realtimeRayTracing<<<blocks, threads>>>(
            CAMERA, d_frmbuffer1, frmW, frmH,
            TxtrMgr.d_txtrFlat, TxtrMgr.d_txtrPtr,
            MtlMgr.d_mtls,
            MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n,
            MeshMgr.d_geom, MeshMgr.gNum,

            BvhMgr.d_gIdx, BvhMgr.d_nodes, BvhMgr.nNum,

            LightMgr.d_lSrc, LightMgr.num,

            d_randState
        );
        cudaDeviceSynchronize();

        // FXAA
        if (hasFXAA)
        {
            // FXAA
            calcLuminance<<<blocks, threads>>>(d_luminance, d_frmbuffer1, frmW, frmH);
            cudaDeviceSynchronize();

            edgeMask<<<blocks, threads>>>(d_edge, d_luminance, frmW, frmH);
            cudaDeviceSynchronize();

            applyFXAAtoBuffer<<<blocks, threads>>>(d_luminance, d_edge, d_frmbuffer1, d_frmbuffer2, frmW, frmH);
            cudaDeviceSynchronize();

            SFTex.updateTexture(d_frmbuffer2, frmW, frmH);
        } else
            SFTex.updateTexture(d_frmbuffer1, frmW, frmH);

        LOG.addLog("Welcome to AsczEngineRT v0", sf::Color::Green, 1);
        LOG.addLog("FPS: " + std::to_string(FPS.fps), sf::Color::Blue);
        LOG.addLog(CAMERA.data(), sf::Color::White, 0);

        // Clear window
        window.clear();

        window.draw(SFTex.sprite);

        // Draw the crosshair
        window.draw(crosshair1);
        window.draw(crosshair2);

        LOG.drawLog(window);

        // For the time being just draw the window
        window.display();

        // Frame end
        FPS.endFrame();
    }

    // Free device memory
    TxtrMgr.freeDevice();
    MtlMgr.freeDevice();
    MeshMgr.freeDevice();
    cudaFree(d_frmbuffer1);
    cudaFree(d_frmbuffer2);
    cudaFree(d_luminance);
    cudaFree(d_edge);

    return 0;
}
