#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Utility.cuh>
#include <random>

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

    // =============== Initialize Important Variables ==============

    // Texture manager
    TxtrManager TxtrMgr;

    // Create SFMLTexture
    int frmW = winW / 2;
    int frmH = winH / 2;
    SFMLTexture SFTex(frmW, frmH);
    SFTex.sprite.setScale(2, 2);

    // Create Camera
    Camera CAMERA;
    CAMERA.pos = Vec3f(0, 5, -10);
    CAMERA.rot = Vec3f(0, 0, 0);
    CAMERA.updateView();

    int threads = 256;
    int blocks = (frmW * frmH + threads - 1) / threads;
    Vec3f *d_framebuffer;
    cudaMalloc(&d_framebuffer, frmW * frmH * sizeof(Vec3f));

    // ======================================================================== 
    // ======================= Some test geometries ===========================
    // ========================================================================

    // Test sphere
    const int m = 2;
    const int n = 2;
    float u = 10.0f;
    float r = 2.0f;
    int count = 0;
    Geom sphs[(2 * m + 1) * (2 * n + 1) * 2];
    for (int x = -m; x <= m; x++) {
        for (int z = -n; z <= n; z++) {
            Vec3f rndColor = Vec3f(
                rand() % 256 / 255.0f,
                rand() % 256 / 255.0f,
                rand() % 256 / 255.0f
            );

            int idx = count++;
            sphs[idx].type = Geom::SPHERE;
            sphs[idx].sph = Sphere(
                Vec3f(x * u, r, z * u), r, rndColor
            );

            // sphs[idx].reflect = 0.7f;
        }
    }

    // Test plane
    Geom pln(Geom::PLANE);
    pln.pln = Plane( Vec3f(0, 1, 0), 0, Vec3f(1) );
    // pln.Fresnel = 0.4f;

    // Test sky
    Geom sky(Geom::SPHERE);
    sky.sph = Sphere( Vec3f(0, 100, 0), 9000.0f, Vec3f(0.5, 0.6, 1) );
    sky.isSky = true;
    sky.txtrIdx = 0;
    TxtrMgr.appendTexture("assets/Textures/Yellow.png"); // Sky texture

    // Test obj
    std::vector<Geom> shape = Utils::readObjFile("test",
        // "assets/Models/Shapes/Cornell/Cornell-Box.obj", 1, 2
        "assets/Models/Shapes/Cube2.obj", 1, 2
    );
    int shapeNum = shape.size();
    for (int i = 0; i < shapeNum; i++) {
        // Custom settings
        shape[i].Fresnel = 0.3f;
    }

    // Test a wall with texture
    Vec2f minXZ = Vec2f(-5, -5);
    Vec2f maxXZ = Vec2f(5, 5); 

    Geom wall1(Geom::TRIANGLE);
    wall1.tri.v0 = Vec3f(minXZ.x, 0, minXZ.y);
    wall1.tri.v1 = Vec3f(maxXZ.x, 0, minXZ.y);
    wall1.tri.v2 = Vec3f(minXZ.x, 0, maxXZ.y);
    wall1.tri.t0 = Vec2f(0, 0);
    wall1.tri.t1 = Vec2f(1, 0);
    wall1.tri.t2 = Vec2f(0, 1);
    wall1.tri.uniformNormal(Vec3f(0, 1, 0));

    Geom wall2(Geom::TRIANGLE);
    wall2.tri.v0 = Vec3f(maxXZ.x, 0, maxXZ.y);
    wall2.tri.v1 = Vec3f(minXZ.x, 0, maxXZ.y);
    wall2.tri.v2 = Vec3f(maxXZ.x, 0, minXZ.y);
    wall2.tri.t0 = Vec2f(1, 1);
    wall2.tri.t1 = Vec2f(0, 1);
    wall2.tri.t2 = Vec2f(1, 0);
    wall2.tri.uniformNormal(Vec3f(0, 1, 0));

    wall1.txtrIdx = 1;
    wall2.txtrIdx = 1;
    TxtrMgr.appendTexture("assets/Textures/Night.png"); // Wall texture 

    TxtrMgr.hostToDevice();

    std::vector<Geom> geoms;
    geoms.push_back(sky);
    geoms.push_back(pln);
    // geoms.push_back(wall1);
    // geoms.push_back(wall2);

    // geoms.insert(geoms.end(), sphs, sphs + count);
    geoms.insert(geoms.end(), shape.begin(), shape.end());

    // ========================================================================
    // ========================================================================

    Geom *d_geoms;
    int geomNum = geoms.size();
    cudaMalloc(&d_geoms, geomNum * sizeof(Geom));
    cudaMemcpy(d_geoms, geoms.data(), geomNum * sizeof(Geom), cudaMemcpyHostToDevice);

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

        // Prepare framebuffer
        clearFrameBuffer<<<blocks, threads>>>(d_framebuffer, frmW, frmH);
        cudaDeviceSynchronize();

        // Render framebuffer
        iterativeRayTracing<<<blocks, threads>>>(
            CAMERA, d_framebuffer,
            d_geoms, geomNum, frmW, frmH,
            TxtrMgr.d_txtrFlat, TxtrMgr.d_txtrPtr, TxtrMgr.txtrCount
        );
        cudaDeviceSynchronize();

        SFTex.updateTexture(d_framebuffer, frmW, frmH);

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

    return 0;
}
