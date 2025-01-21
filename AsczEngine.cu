#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <Camera.cuh>
#include <SFMLTexture.cuh>

#include <Utility.cuh>
#include <random>


__global__ void clearFrameBuffer(Vec3f *framebuffer, int width, int height) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < width * height) framebuffer[i] = Vec3f(0, 0, 0);
}

__global__ void renderFrameBuffer(
    Vec3f *framebuffer, Camera camera, Geom *geoms, int geomNum, int width, int height
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= width * height) return;

    int x = i % width;
    int y = i / width;

    Ray primaryRay = camera.castRay(x, y, width, height);

    // Iterative ray tracing

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

        for (int g = 0; g < geomNum; g++) {
            const Geom &geom = geoms[g];

            switch (geom.type) {
                case Geom::TRIANGLE: {
                    const Triangle &tri = geom.triangle;
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
                    }
                    continue;
                }

                case Geom::SPHERE: {
                    const Sphere &sph = geom.sphere;

                    Vec3f l = sph.o - ray.origin;
                    float tca = l * ray.direction;
                    float d2 = l * l - tca * tca;

                    if (d2 > sph.r * sph.r) continue;

                    float thc = sqrt(sph.r * sph.r - d2);
                    float t0 = tca - thc;
                    float t1 = tca + thc;

                    if (t0 < 0) t0 = t1;

                    if (t0 > 0.00001 && t0 < hit.t) {
                        hit.hit = true;
                        hit.idx = g;
                        hit.t = t0;
                    }

                    continue;
                }

                case Geom::PLANE: {
                    const Plane &pln = geom.plane;

                    float denom = pln.n * ray.direction;
                    if (denom > -0.00001 && denom < 0.00001) continue; // Parallel

                    float t = -(pln.n * ray.origin + pln.d) / denom;

                    if (t < hit.t && t > 0.00001) {
                        hit.hit = true;
                        hit.idx = g;
                        hit.t = t;
                    }
                    continue;
                }
            }
        }

    // =========================================================================
    // =========================================================================
    // =========================================================================

        if (!hit.hit) continue;

        // Interpolate the hit point
        const Geom &geom = geoms[hit.idx];

        vrtx[r] = ray.origin + ray.direction * hit.t;

        switch (geom.type) {
            case Geom::TRIANGLE: {
                const Triangle &tri = geom.triangle;
                float w = 1 - hit.u - hit.v;

                colr[r] = tri.c0 * w + tri.c1 * hit.u + tri.c2 * hit.v;
                nrml[r] = tri.n0 * w + tri.n1 * hit.u + tri.n2 * hit.v;
                nrml[r].norm();
                break;
            }

            case Geom::SPHERE: {
                const Sphere &sph = geom.sphere;
                colr[r] = sph.color;
                nrml[r] = (vrtx[r] - sph.o) / sph.r;
                if (sph.invert) nrml[r] = -nrml[r];
                break;
            }

            case Geom::PLANE: {
                const Plane &pln = geom.plane;
                colr[r] = pln.color;
                nrml[r] = pln.n;
                break;
            }
        }

        // If the geometry is a sky, ignore the rest
        if (geom.isSky) continue;

        Vec3f lightSrc(0, 10, 30);

        // Shadow ray
        Vec3f lightDir = lightSrc - vrtx[r];
        Vec3f lightOrigin = vrtx[r] + lightDir * 0.0001;
        lightDir.norm();
        Ray shadowRay(lightOrigin, lightDir);
        bool shadow = false;

        for (int g = 0; g < geomNum; g++) {
            const Geom &geom = geoms[g];
            if (geom.isSky) continue; // Ignore sky

            switch (geom.type) {
                case Geom::TRIANGLE: {
                    const Triangle &tri = geom.triangle;
                    Vec3f e1 = tri.v1 - tri.v0;
                    Vec3f e2 = tri.v2 - tri.v0;
                    Vec3f h = shadowRay.direction & e2;
                    float a = e1 * h;

                    if (a > -0.00001 && a < 0.00001) continue;

                    float f = 1.0f / a;
                    Vec3f s = shadowRay.origin - tri.v0;
                    float u = f * (s * h);

                    if (u < 0.0f || u > 1.0f) continue;

                    Vec3f q = s & e1;
                    float v = f * (shadowRay.direction * q);

                    if (v < 0.0f || u + v > 1.0f) continue;

                    float t = f * (e2 * q);

                    if (t > 0.00001) {
                        shadow = true;
                        break;
                    }
                    continue;
                }

                case Geom::SPHERE: {
                    const Sphere &sph = geom.sphere;

                    Vec3f l = sph.o - shadowRay.origin;
                    float tca = l * shadowRay.direction;
                    float d2 = l * l - tca * tca;

                    if (d2 > sph.r * sph.r) continue;

                    float thc = sqrt(sph.r * sph.r - d2);
                    float t0 = tca - thc;
                    float t1 = tca + thc;

                    if (t0 < 0) t0 = t1;

                    if (t0 > 0.00001) {
                        shadow = true;
                        break;
                    }

                    continue;
                }

                case Geom::PLANE: {
                    const Plane &pln = geom.plane;

                    float denom = pln.n * shadowRay.direction;
                    if (denom > -0.00001 && denom < 0.00001) continue; // Parallel

                    float t = -(pln.n * shadowRay.origin + pln.d) / denom;

                    if (t > 0.00001) {
                        shadow = true;
                        break;
                    }
                    continue;
                }
            }
        }

        if (shadow) weights[r] *= 0.3;

        // Apply very basic lighting with light ray from the top
        float diff = nrml[r] * lightDir;
        if (diff < 0) diff = 0;

        diff = 0.3 + diff * 0.7;
        colr[r] *= diff;

        if (geom.reflect > 0.0f) {
            float weightLeft = weights[r] * geom.reflect;
            weights[r] *= (1 - geom.reflect);

            Vec3f reflDir = ray.reflect(nrml[r]);
            Vec3f reflOrigin = vrtx[r] + nrml[r] * 0.0001;

            rays[++rnum] = Ray(reflOrigin, reflDir);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (geom.transmit > 0.0f) {
            float weightLeft = weights[r] * geom.transmit;
            weights[r] *= (1 - geom.transmit);

            Vec3f transOrg = vrtx[r] + ray.direction * 0.0001;

            rays[++rnum] = Ray(transOrg, ray.direction);
            hits[rnum] = RayHit();
            weights[rnum] = weightLeft;
        }
        else if (geom.Fresnel > 0.0f) {
            float weightLeft = weights[r] * geom.Fresnel;
            weights[r] *= (1 - geom.Fresnel);

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

    // ======================================================================== 
    // ======================= Some test geometries ===========================
    // ========================================================================

    // Test sphere
    const int m = 2;
    const int n = 2;
    float u = 6.0f;
    float r = 2.0f;
    int count = 0;
    Geom sph[(2 * m + 1) * (2 * n + 1)];
    for (int x = -m; x <= m; x++) {
        for (int z = -n; z <= n; z++) {
            Vec3f rndColor = Vec3f(
                rand() % 256 / 255.0f,
                rand() % 256 / 255.0f,
                rand() % 256 / 255.0f
            );

            int idx = count++;
            sph[idx].type = Geom::SPHERE;
            sph[idx].sphere = Sphere( Vec3f(x * u, r, z * u), r, rndColor );

            sph[idx].reflect = 0.7f;
        }
    }

    // Test plane
    Geom pln;
    pln.type = Geom::PLANE;
    pln.plane = Plane( Vec3f(0, 1, 0), 0, Vec3f(1) );
    pln.Fresnel = 0.2f;

    // Test sky
    Geom sky;
    sky.type = Geom::SPHERE;
    sky.sphere = Sphere( Vec3f(0), 1000.0f, Vec3f(0.5, 0.6, 1) );
    sky.isSky = true;

    // // Test obj
    // std::vector<Geom> shape = Utils::readObjFile("test",
    //     "assets/Models/Shapes/Cube1.obj", 1, 0
    // );
    // int shapeNum = shape.size();

    std::vector<Geom> geoms;
    geoms.push_back(sky);
    geoms.push_back(pln);
    for (int i = 0; i < count; i++) geoms.push_back(sph[i]);

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
                sf::Mouse::setPosition(sf::Vector2i(width / 2, height / 2), window);

                // Hide cursor
                window.setMouseCursorVisible(!CAMERA.focus);
            }

            // Scroll to change the field of view
            if (event.type == sf::Event::MouseWheelScrolled) {
                if (event.mouseWheelScroll.delta > 0) CAMERA.fov += 0.1;
                else if (event.mouseWheelScroll.delta < 0) CAMERA.fov -= 0.1;
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
        renderFrameBuffer<<<blocks, threads>>>(d_framebuffer, CAMERA, d_geoms, geomNum, width, height);
        cudaDeviceSynchronize();

        SFTex.updateTexture(d_framebuffer, width, height);

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
