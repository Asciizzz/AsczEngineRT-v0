#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Camera.cuh>
#include <SFMLTexture.cuh>
#include <Utility.cuh>

struct RayHit {
    bool hit = false;
    int idx = -1;
    float t = 1e8;
    float u = 0;
    float v = 0;
    float w = 0;
    Vec3f vrtx;
    Vec2f txtr;
    Vec3f nrml;
    Vec3f colr;
};

struct RayContribution {
    Ray ray;
    RayHit hit;
};

__device__ RayHit iterativeRayTracing(
    const Ray &ray, const Triangle *triangles, int triNum
) {
    RayContribution contributions[20];
    int contributionCount = 0;
    bool lastHit = false;

    while (!lastHit && contributionCount < 20) {
        RayHit hit = RayHit();

        // Will be replaced with BVH
        for (int i = 0; i < triNum; i++) {
            const Triangle &tri = triangles[i];
            Vec3f e1 = tri.v1 - tri.v0;
            Vec3f e2 = tri.v2 - tri.v0;
            Vec3f h = ray.direction & e2;
            float a = e1 * h;

            if (a > -0.00001 && a < 0.00001) continue;

            float f = 1.0f / a;
            Vec3f s = ray.origin - tri.v0;
            float u = f * (s * h);

            if (u < 0.0f || u > 1.0f) continue;

            Vec3f q = s & e1;
            float v = f * (ray.direction * q);

            if (v < 0.0f || u + v > 1.0f) continue;

            float t = f * (e2 * q);

            if (t > 0.00001 && t < hit.t) {
                hit.hit = true;
                hit.idx = i;
                hit.t = t;
                hit.u = u;
                hit.v = v;
                hit.w = 1 - u - v;
            }
        }

        if (!hit.hit) {
            lastHit = true;
            break;
        }

        Triangle tri = triangles[hit.idx];

        hit.vrtx = ray.origin + ray.direction * hit.t;
        hit.nrml = tri.n0 * hit.w + tri.n1 * hit.u + tri.n2 * hit.v;
        hit.colr = tri.c0 * hit.w + tri.c1 * hit.u + tri.c2 * hit.v;

        contributions[contributionCount++] = {ray, hit};

        // For the time being just break the loop
        lastHit = true;
    }

    if (contributionCount == 0) return RayHit();

    RayHit finalHit = contributions[0].hit;
    return finalHit;
}


__global__ void clearFrameBuffer(Vec3f *framebuffer, int width, int height) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < width * height) framebuffer[i] = Vec3f(0, 0, 0);
}

__global__ void renderFrameBuffer(
    Vec3f *framebuffer, Camera camera, Triangle *triangles, int triNum, int width, int height
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= width * height) return;

    int x = i % width;
    int y = i / width;

    Ray ray = camera.castRay(x, y, width, height);
    RayHit hit = iterativeRayTracing(ray, triangles, triNum);

    if (hit.hit) framebuffer[i] = hit.colr;
}

int main() {
    FpsHandler &FPS = FpsHandler::instance();
    CsLogHandler LOG = CsLogHandler();

    cudaFree(0);  // Force context initialization
    cudaDeviceSetLimit(cudaLimitStackSize, 256 * 1024);

    // Create SFMLTexture
    int width = 1600;
    int height = 900;
    SFMLTexture SFTex(width, height);

    // Test camera
    Camera CAMERA;
    CAMERA.pos = Vec3f(0, 0, -10);
    CAMERA.rot = Vec3f(0, 0, 0);
    CAMERA.updateView();

    // Create window
    sf::RenderWindow window(sf::VideoMode(width, height), "AsczEngine");
    sf::Mouse::setPosition(sf::Vector2i(width / 2, height / 2), window);
    window.setMouseCursorVisible(!CAMERA.focus);

    // Crosshair
    int crosshairSize = 10;
    int crosshairThick = 2;
    sf::Color crosshairColor = sf::Color::Green;
    sf::RectangleShape crosshair1(
        sf::Vector2f(crosshairSize + crosshairThick, crosshairThick)
    );
    crosshair1.setPosition(width / 2 - crosshairSize / 2, height / 2);
    crosshair1.setFillColor(crosshairColor);
    sf::RectangleShape crosshair2(
        sf::Vector2f(crosshairThick, crosshairSize + crosshairThick)
    );
    crosshair2.setPosition(width / 2, height / 2 - crosshairSize / 2);
    crosshair2.setFillColor(crosshairColor);

    int threads = 256;
    int blocks = (width * height + threads - 1) / threads;
    Ray *d_rays;
    cudaMalloc(&d_rays, width * height * sizeof(Ray));
    Vec3f *d_framebuffer;
    cudaMalloc(&d_framebuffer, width * height * sizeof(Vec3f));

    // Some test triangles
    Triangle tri1;
    tri1.v0 = Vec3f(-10, -10, -15);
    tri1.v1 = Vec3f(10, -10, -15);
    tri1.v2 = Vec3f(0, 10, -15);
    tri1.c0 = Vec3f(1, 0.6, 0.6);
    tri1.c1 = Vec3f(0.6, 1, 0.6);
    tri1.c2 = Vec3f(0.6, 0.6, 1);
    tri1.uniformNormal(Vec3f(0, 0, 1));
    tri1.normAll();
    tri1.reflect = 0.1;

    Triangle tri2;
    tri2.v0 = Vec3f(-8, -8, 15);
    tri2.v1 = Vec3f(8, -8, 15);
    tri2.v2 = Vec3f(0, 8, 15);
    tri2.uniformColor(Vec3f(0, 0, 1));
    tri2.uniformNormal(Vec3f(0, 0, -1));
    tri2.normAll();

    int triNum = 2;
    Triangle *d_triangles;
    cudaMalloc(&d_triangles, triNum * sizeof(Triangle));
    cudaMemcpy(d_triangles, &tri1, sizeof(Triangle), cudaMemcpyHostToDevice);
    cudaMemcpy(d_triangles + 1, &tri2, sizeof(Triangle), cudaMemcpyHostToDevice);

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
                sf::Mouse::setPosition(sf::Vector2i(width / 2, height / 2), window);

                // Hide cursor
                window.setMouseCursorVisible(!CAMERA.focus);
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
            sf::Mouse::setPosition(sf::Vector2i(
                width / 2, height / 2
            ), window);

            // Move from center
            int dMx = mousepos.x - width / 2;
            int dMy = mousepos.y - height / 2;

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
        clearFrameBuffer<<<blocks, threads>>>(d_framebuffer, width, height);
        cudaDeviceSynchronize();

        // Render framebuffer
        renderFrameBuffer<<<blocks, threads>>>(d_framebuffer, CAMERA, d_triangles, triNum, width, height);
        cudaDeviceSynchronize();

        SFTex.updateTexture(d_framebuffer, width, height, 1);

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
