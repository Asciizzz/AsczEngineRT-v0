#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Camera.cuh>
#include <SFMLTexture.cuh>

#include <Utility.cuh>

__device__ Vec3f iterativeRayTracing(
    const Ray &primaryRay, const Triangle *triangles, int triNum
) {
    Ray rays[20] = { primaryRay };
    RayHit hits[20] = { RayHit() };
    Vec3f vrtx[20], nrml[20], colr[20];
    float weights[20] = { 1.0f };

    int rnum = 0;

    for (int r = 0; r < rnum + 1; r++) {
        if (rnum > 16) break;

        Ray &ray = rays[r];
        RayHit &hit = hits[r];

// =========================================================================
// ======================= Will be replaced with BVH =======================
// =========================================================================

        for (int g = 0; g < triNum; g++) {
            const Triangle &tri = triangles[g];
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
                hit.idx = g;
                hit.t = t;
                hit.u = u;
                hit.v = v;
                hit.w = 1 - u - v;
            }
        }

// =========================================================================
// =========================================================================
// =========================================================================

        if (!hit.hit) continue;

        Triangle tri = triangles[hit.idx];

        vrtx[r] = ray.origin + ray.direction * hit.t;
        colr[r] = tri.c0 * hit.w + tri.c1 * hit.u + tri.c2 * hit.v;
        nrml[r] = tri.n0 * hit.w + tri.n1 * hit.u + tri.n2 * hit.v;
        nrml[r].norm();

        if (tri.reflect > 0.0f) {
            float weightLeft = weights[r] * tri.reflect;
            weights[r] *= (1 - tri.reflect);

            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * 0.0001;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (tri.transmit > 0.0f) {
            float weightLeft = weights[r] * tri.transmit;
            weights[r] *= (1 - tri.transmit);

            Vec3f transOrg = vrtx[r] + ray.direction * 0.0001;

            rays[++rnum] = Ray(transOrg, ray.direction);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (tri.Fresnel > 0.0f) {
            float weightLeft = weights[r] * tri.Fresnel;
            weights[r] *= (1 - tri.Fresnel);

            // Schlick's approximation
            float cosI = (-ray.direction) * nrml[r];
            if (cosI < 0) cosI = -cosI;

            // Find the fresnel coefficient
            float R = pow(1 - cosI, 5);
            float Rrefl = R * weightLeft;
            float Rrefr = (1 - R) * weightLeft;

            // Refraction (for the time being just tranparent)
            Vec3f refrDir = ray.direction;
            Vec3f refrOrigin = vrtx[r] + refrDir * 0.0001;

            rays[++rnum] = Ray(refrOrigin, refrDir);
            hits[rnum] = RayHit();
            weights[rnum] = Rrefr;

            // Reflection
            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * 0.0001;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = Rrefl;
        }
    }

    Vec3f finalColr(0, 0, 0);
    for (int i = 0; i <= rnum; i++) {
        finalColr += colr[i] * weights[i];
    }

    return finalColr;
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
    Vec3f finalColr = iterativeRayTracing(ray, triangles, triNum);

    framebuffer[i] = finalColr;
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
    Vec3f *d_framebuffer;
    cudaMalloc(&d_framebuffer, width * height * sizeof(Vec3f));

    // Some test triangles
    Triangle tris[12];
    float boxXw = 20; // X width (x2)
    float boxYh = 10; // Y height (x2)
    float boxZw = 20; // Z width (x2)

    // Front
    tris[0].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[0].v1 = Vec3f(boxXw, -boxYh, -boxZw);
    tris[0].v2 = Vec3f(boxXw, boxYh, -boxZw);
    tris[0].uniformColor(Vec3f(1, 0, 0));
    tris[0].uniformNormal(Vec3f(0, 0, 1));
    tris[0].normAll();

    tris[1].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[1].v1 = Vec3f(boxXw, boxYh, -boxZw);
    tris[1].v2 = Vec3f(-boxXw, boxYh, -boxZw);
    tris[1].uniformColor(Vec3f(1, 0, 0));
    tris[1].uniformNormal(Vec3f(0, 0, 1));
    tris[1].normAll();

    // Back
    tris[2].v0 = Vec3f(-boxXw, -boxYh, boxZw);
    tris[2].v1 = Vec3f(boxXw, -boxYh, boxZw);
    tris[2].v2 = Vec3f(boxXw, boxYh, boxZw);
    tris[2].uniformColor(Vec3f(0, 1, 0));
    tris[2].uniformNormal(Vec3f(0, 0, -1));
    tris[2].normAll();

    tris[3].v0 = Vec3f(-boxXw, -boxYh, boxZw);
    tris[3].v1 = Vec3f(boxXw, boxYh, boxZw);
    tris[3].v2 = Vec3f(-boxXw, boxYh, boxZw);
    tris[3].uniformColor(Vec3f(0, 1, 0));
    tris[3].uniformNormal(Vec3f(0, 0, -1));
    tris[3].normAll();

    // Left
    tris[4].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[4].v1 = Vec3f(-boxXw, -boxYh, boxZw);
    tris[4].v2 = Vec3f(-boxXw, boxYh, boxZw);
    tris[4].uniformColor(Vec3f(0, 0, 1));
    tris[4].uniformNormal(Vec3f(1, 0, 0));
    tris[4].normAll();

    tris[5].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[5].v1 = Vec3f(-boxXw, boxYh, boxZw);
    tris[5].v2 = Vec3f(-boxXw, boxYh, -boxZw);
    tris[5].uniformColor(Vec3f(0, 0, 1));
    tris[5].uniformNormal(Vec3f(1, 0, 0));
    tris[5].normAll();

    // Right
    tris[6].v0 = Vec3f(boxXw, -boxYh, -boxZw);
    tris[6].v1 = Vec3f(boxXw, -boxYh, boxZw);
    tris[6].v2 = Vec3f(boxXw, boxYh, boxZw);
    tris[6].uniformColor(Vec3f(1, 1, 0));
    tris[6].uniformNormal(Vec3f(-1, 0, 0));
    tris[6].normAll();
    
    tris[7].v0 = Vec3f(boxXw, -boxYh, -boxZw);
    tris[7].v1 = Vec3f(boxXw, boxYh, boxZw);
    tris[7].v2 = Vec3f(boxXw, boxYh, -boxZw);
    tris[7].uniformColor(Vec3f(1, 1, 0));
    tris[7].uniformNormal(Vec3f(-1, 0, 0));
    tris[7].normAll();

    // Top (ceiling)
    tris[8].v0 = Vec3f(-boxXw, boxYh, -boxZw);
    tris[8].v1 = Vec3f(boxXw, boxYh, -boxZw);
    tris[8].v2 = Vec3f(boxXw, boxYh, boxZw);
    tris[8].uniformColor(Vec3f(1, 0, 1));
    tris[8].uniformNormal(Vec3f(0, -1, 0));
    tris[8].normAll();

    tris[9].v0 = Vec3f(-boxXw, boxYh, -boxZw);
    tris[9].v1 = Vec3f(boxXw, boxYh, boxZw);
    tris[9].v2 = Vec3f(-boxXw, boxYh, boxZw);
    tris[9].uniformColor(Vec3f(1, 0, 1));
    tris[9].uniformNormal(Vec3f(0, -1, 0));
    tris[9].normAll();

    // Bottom (floor)
    tris[10].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[10].v1 = Vec3f(boxXw, -boxYh, -boxZw);
    tris[10].v2 = Vec3f(boxXw, -boxYh, boxZw);
    tris[10].uniformColor(Vec3f(1, 1, 1));
    tris[10].uniformNormal(Vec3f(0, 1, 0));
    tris[10].normAll();

    tris[11].v0 = Vec3f(-boxXw, -boxYh, -boxZw);
    tris[11].v1 = Vec3f(boxXw, -boxYh, boxZw);
    tris[11].v2 = Vec3f(-boxXw, -boxYh, boxZw);
    tris[11].uniformColor(Vec3f(1, 1, 1));
    tris[11].uniformNormal(Vec3f(0, 1, 0));
    tris[11].normAll();

    // Fun floor stuff
    tris[10].Fresnel = 0.2f;
    tris[11].Fresnel = 0.2f;

    // int triNum = 12;
    // Triangle *d_triangles;
    // cudaMalloc(&d_triangles, triNum * sizeof(Triangle));
    // cudaMemcpy(d_triangles, tris, triNum * sizeof(Triangle), cudaMemcpyHostToDevice);

    // Test obj
    std::vector<Triangle> trisObj = Utils::readObjFile("test",
        "assets/Models/Shapes/Cube1.obj", 1, 0
    );
    int triNum = trisObj.size();
    Triangle *d_triangles;
    cudaMalloc(&d_triangles, triNum * sizeof(Triangle));
    cudaMemcpy(d_triangles, trisObj.data(), triNum * sizeof(Triangle), cudaMemcpyHostToDevice);

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

        SFTex.updateTexture(d_framebuffer, width, height);

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
