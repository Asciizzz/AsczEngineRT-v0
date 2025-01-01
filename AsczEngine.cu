#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <cuda_runtime.h>

#include <Camera.cuh>
#include <SFMLTexture.cuh>

/* Goal:

Ray intersection with AABB: Done
Ray reflection: Done
Create rays from camera: Done
Ray casting:

*/

struct Triangle {
    Vec3f v0, v1, v2;
    // Vec2f t0, t1, t2; // Will be ignored for now
    Vec3f c1, c2, c3; // Placeholder, to test interpolation
    Vec3f n0, n1, n2;

    bool reflect = false; // Test reflection
};

__global__ void clearFramebuffer(Vec3f *framebuffer, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    framebuffer[idx] = Vec3f(0, 0, 0);
}

__global__ void generateRays(Camera camera, Ray *rays, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    int x = idx % width;
    int y = idx / width;

    rays[y * width + x] = camera.castRay(x, y, width, height);
}

__global__ void castRays(
    Vec3f *framebuffer, Ray *rays, bool *raycursive,
    Triangle *triangles, int width, int height, int triangleCount) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    Ray ray = rays[idx];

    Ray recursiveRay;
    bool recursive = false;

    float zdepth = 1000000.0f;
    // This will soon be replaced with BVH traversal method
    for (int i = 0; i < triangleCount; i++) {
        Vec3f A = triangles[i].v0;  
        Vec3f B = triangles[i].v1;
        Vec3f C = triangles[i].v2;

        Vec3f e1 = B - A;
        Vec3f e2 = C - A;

        Vec3f h = ray.direction & e2;
        float a = e1 * h;

        // Ray is parallel to the triangle
        if (a > -0.00001 && a < 0.00001) continue;

        float f = 1.0f / a;
        Vec3f s = ray.origin - A;
        float u = f * (s * h);

        if (u < 0.0f || u > 1.0f) continue;

        Vec3f q = s & e1;
        float v = f * (ray.direction * q);

        if (v < 0.0f || u + v > 1.0f) continue;

        float t = f * (e2 * q);

        if (t > 0.00001 && t < zdepth) {
            zdepth = t;

            // If reflective, the resulting ray will be the reflection of the current ray
            if (triangles[i].reflect) {
                recursiveRay.origin = ray.origin + ray.direction * t;
                recursiveRay.direction = ray.reflect((B - A) & (C - A));
                recursive = true;

                continue;
            }

            recursive = false;

            // Interpolate color
            Vec3f color = triangles[i].c1 * (1 - u - v) + triangles[i].c2 * u + triangles[i].c3 * v;
            framebuffer[idx] = color;
            // Interpolate normal
            Vec3f normal = triangles[i].n0 * (1 - u - v) + triangles[i].n1 * u + triangles[i].n2 * v;
        }
    }

    if (recursive) {
        rays[idx] = recursiveRay;
        atmoicAdd(raycursive, 1);
    }
}   

int main() {
    /*
    Isn't it funny how these things have been here
    For 3 entire versions of the engine?
    They been through 3 "code wars"
    */
    FpsHandler &FPS = FpsHandler::instance();
    CsLogHandler LOG = CsLogHandler();

    // Create SFMLTexture
    int width = 600;
    int height = 600;
    SFMLTexture SFTex(600, 600);

    // Test camera
    Camera CAMERA;
    CAMERA.pos = Vec3f(0, 0, 0);
    CAMERA.rot = Vec3f(0, 0, 0);
    CAMERA.updateView();

    Ray *d_rays;
    Vec3f *d_framebuffer;
    cudaMalloc(&d_rays, width * height * sizeof(Ray));
    cudaMalloc(&d_framebuffer, width * height * sizeof(Vec3f));

    // For ray recursion
    Ray *d_raycursive; // Pun intended
    bool *d_hasreflected;
    cudaMalloc(&d_raycursive, width * height * sizeof(Ray));
    cudaMalloc(&d_hasreflected, sizeof(bool));

    int threads = 256;
    int blocks = (width * height + threads - 1) / threads;

    // Creating some test triangles
    Triangle *d_triangles;
    int triCount = 4;
    cudaMalloc(&d_triangles, triCount * sizeof(Triangle));

    Triangle triangles[4];
    // Postive Z
    triangles[0].v0 = Vec3f(-10, -10, 50);
    triangles[0].v1 = Vec3f(10, -10, 50);
    triangles[0].v2 = Vec3f(0, 10, 50);
    triangles[0].c1 = Vec3f(1, 0, 0);   
    triangles[0].c2 = Vec3f(0, 1, 0);
    triangles[0].c3 = Vec3f(0, 0, 1);

    // Negative Z
    triangles[1].v0 = Vec3f(-10, -10, -50);
    triangles[1].v1 = Vec3f(10, -10, -50);
    triangles[1].v2 = Vec3f(0, 10, -50);
    triangles[1].c1 = Vec3f(1, 0, 0);
    triangles[1].c2 = Vec3f(1, 1, 0);
    triangles[1].c3 = Vec3f(0, 1, 0);
    triangles[1].reflect = true;

    // Positive X
    triangles[2].v0 = Vec3f(50, -10, -10);
    triangles[2].v1 = Vec3f(50, -10, 10);
    triangles[2].v2 = Vec3f(50, 10, 0);
    triangles[2].c1 = Vec3f(0, 1, 0);
    triangles[2].c2 = Vec3f(0, 0, 1);
    triangles[2].c3 = Vec3f(1, 0, 0);

    // Negative X
    triangles[3].v0 = Vec3f(-50, -10, -10);
    triangles[3].v1 = Vec3f(-50, -10, 10);
    triangles[3].v2 = Vec3f(-50, 10, 0);
    triangles[3].c1 = Vec3f(0, 1, 0);
    triangles[3].c2 = Vec3f(0, 0, 1);
    triangles[3].c3 = Vec3f(1, 0, 0);

    cudaMemcpy(d_triangles, triangles, triCount * sizeof(Triangle), cudaMemcpyHostToDevice);   

    sf::RenderWindow window(sf::VideoMode(width, height), "AsczEngine");

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
            CAMERA.rot.y -= dMx * CAMERA.mSens * FPS.dTimeSec;

        // Csgo perspective mode movement
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

        // Clear framebuffer
        clearFramebuffer<<<blocks, threads>>>(d_framebuffer, width, height);    
        cudaDeviceSynchronize();

        // Generate rays
        generateRays<<<blocks, threads>>>(CAMERA, d_rays, width, height);
        cudaDeviceSynchronize();

        // Cast rays
        castRays<<<blocks, threads>>>(d_framebuffer, d_rays, d_triangles, width, height, triCount);
        cudaDeviceSynchronize();

        // Log
        LOG.addLog("Welcome to AsczEngine RTx", sf::Color::White, 1);
        LOG.addLog("FPS: " + std::to_string(FPS.fps), sf::Color::White);

        // Draw to window
        SFTex.updateTexture(d_framebuffer, width, height, 1);
        window.clear(sf::Color::Black);
        window.draw(SFTex.sprite);
        LOG.drawLog(window);
        window.display();

        // Frame end
        FPS.endFrame();
    }

    return 0;
}