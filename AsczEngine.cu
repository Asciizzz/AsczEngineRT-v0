#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Utility.cuh>
#include <random>

#include <TxtrManager.cuh>
#include <MatManager.cuh>
#include <MeshManager.cuh>
#include <BvhManager.cuh>

#include <FXAA.cuh>

#include <RayTrace.cuh>
#include <SFMLTexture.cuh>

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
    BvhManager BvhMgr;
    MeshManager MeshMgr;
    TxtrManager TxtrMgr;
    MatManager MatMgr;
    MatMgr.appendMaterial(Material()); // Default material

    // Create Camera
    // By logic, then this is CameraManager?
    // Idk, just a funny thought
    Camera CAMERA;

    // Set frame buffer properties
    float frmScl = 1;
    int frmW = winW / frmScl;
    int frmH = winH / frmScl;

    // Some debugging values

    Vec3f lightSrc = Vec3f(0, 10, 0);

    bool hasFXAA = true;

    // ====================== Some very scuffed init ==========================
    
    std::ifstream cfgFile(".cfg");
    std::string cfgLine;
    while (std::getline(cfgFile, cfgLine)) {
        if (cfgLine.size() == 0 || cfgLine[0] == '#') continue;

        std::stringstream ss(cfgLine);
        std::string type; ss >> type;

        if (type == "CameraPos") {
            ss >> CAMERA.pos.x >> CAMERA.pos.y >> CAMERA.pos.z;
        }
        else if (type == "CameraRot") {
            ss >> CAMERA.rot.x >> CAMERA.rot.y >> CAMERA.rot.z;
        }
        else if (type == "CameraFov") {
            ss >> CAMERA.fov;
        }

        if (type == "LightSrc") {
            ss >> lightSrc.x >> lightSrc.y >> lightSrc.z;
        }

        if (type == "FrameScl") {
            ss >> frmScl;
            frmW = winW / frmScl;
            frmH = winH / frmScl;
        }

        if (type == "FXAA") {
            ss >> hasFXAA;
        }
    };

    // Allocate frame buffer
    int threads = 256;
    int blocks = (frmW * frmH + threads - 1) / threads;
    Vec3f *d_frmbuffer1, *d_frmbuffer2;
    cudaMalloc(&d_frmbuffer1, frmW * frmH * sizeof(Vec3f));
    cudaMalloc(&d_frmbuffer2, frmW * frmH * sizeof(Vec3f));

    // Allocate luminance buffer
    float *d_luminance; bool *d_edge;
    cudaMalloc(&d_luminance, frmW * frmH * sizeof(float));
    cudaMalloc(&d_edge, frmW * frmH * sizeof(bool));

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
        short objPlacement;
        float objScale;

        ss >> objPath >> objPlacement >> objScale;

        Utils::appendObj(
            MeshMgr, MatMgr, TxtrMgr,
            objPath.c_str(), objPlacement, objScale
        );
    }

    // ================ Terrible attemp at building a BVH =====================

    // NodeHst *node = new NodeHst();
    // BvhManager::buildBvh(
    //     node, MeshMgr.ABmin, MeshMgr.ABmax,
    //     MeshMgr.h_v, MeshMgr.h_fv, MeshMgr.h_fi
    // );

    // ======================= Copy to device memory ==========================

    // Copy to device memory
    MeshMgr.hostToDevice();
    TxtrMgr.hostToDevice();
    MatMgr.hostToDevice();

    // ========================================================================
    // ========================================================================

    Vec3f prevPos = CAMERA.pos + 1;
    Vec3f prevRot = CAMERA.rot + 1;

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

            // Press f1 to toggle camera focus
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::F1) {
                CAMERA.focus = !CAMERA.focus;
                // To avoid sudden camera movement when changing focus
                sf::Mouse::setPosition(sf::Vector2i(winW / 2, winH / 2), window);

                // Hide cursor
                window.setMouseCursorVisible(!CAMERA.focus);
            }

            // Scroll to change the field of view
            if (event.type == sf::Event::MouseWheelScrolled) {
                if (event.mouseWheelScroll.delta > 0) CAMERA.fov += 0.1;
                else if (event.mouseWheelScroll.delta < 0) CAMERA.fov -= 0.1;

                if (CAMERA.fov < 0.1) CAMERA.fov += 0.1;
                if (CAMERA.fov > M_PI - 0.1) CAMERA.fov -= 0.1;
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

        if (prevPos != CAMERA.pos || prevRot != CAMERA.rot)
        {
            // Prepare frmbuffer
            clearFrameBuffer<<<blocks, threads>>>(d_frmbuffer1, frmW, frmH);
            cudaDeviceSynchronize();

            // Render frmbuffer
            iterativeRayTracing<<<blocks, threads>>>(
                CAMERA, d_frmbuffer1, frmW, frmH,
                TxtrMgr.d_txtrFlat, TxtrMgr.d_txtrPtr,
                MatMgr.d_mats,
                MeshMgr.d_v, MeshMgr.d_t, MeshMgr.d_n,
                MeshMgr.d_fv, MeshMgr.d_ft, MeshMgr.d_fn, MeshMgr.d_fm,
                MeshMgr.fNum,

                lightSrc
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
        }

        prevPos = CAMERA.pos;
        prevRot = CAMERA.rot;

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
    cudaFree(d_frmbuffer1);
    cudaFree(d_frmbuffer2);
    TxtrMgr.freeDevice();
    MatMgr.freeDevice();
    MeshMgr.freeDevice();

    return 0;
}
