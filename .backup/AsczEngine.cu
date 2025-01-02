#include <FpsHandler.cuh>
#include <CsLogHandler.cuh>

#include <cuda_runtime.h>

#include <Camera.cuh>
#include <SFMLTexture.cuh>
#include <Utility.cuh>

/* Goal:

Ray intersection with AABB: Done
Ray reflection: Done
Create rays from camera: Done
Ray casting: Done
Ray recursion: Done

*/

__global__ void clearFramebuffer(Vec3f *framebuffer, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    framebuffer[idx] = Vec3f(0, 0, 0);
}

__global__ void resetRecursive(bool *raycursive, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    raycursive[idx] = true; // To kickstart the first iteration
}

__global__ void generateRays(Camera camera, Ray *rays, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    int x = idx % width;
    int y = idx / width;

    rays[y * width + x] = camera.castRay(x, y, width, height);
}

__global__ void castRays(
    Vec3f *framebuffer, Ray *rays, bool *raycursive, bool *hasrecursive,
    Triangle *triangles, int width, int height, int triangleCount
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    if (!raycursive[idx]) return;

    Ray ray = rays[idx];

    Ray recursiveRay;
    bool recursive = false;

    Vec3f resultColor = Vec3f(0, 0, 0);

    float zdepth = 1000000.0f;
    // This will soon be replaced with BVH traversal method
    for (int i = 0; i < triangleCount; i++) {
        if (!triangles[i].display) continue;

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

            // Interpolate color
            Vec3f color = triangles[i].c1 * (1 - u - v) + triangles[i].c2 * u + triangles[i].c3 * v;
            // Interpolate normal
            Vec3f normal = triangles[i].n0 * (1 - u - v) + triangles[i].n1 * u + triangles[i].n2 * v;

            // If reflective, the resulting ray will be the reflection of the current ray
            if (triangles[i].reflect) {
                recursiveRay.origin = ray.origin + ray.direction * t + normal * 1e-4;
                recursiveRay.direction = ray.reflect(normal);
                recursive = true;

                continue;
            }

            recursive = false;
            resultColor = color;
        }
    }

    if (recursive) {
        rays[idx] = recursiveRay;
        raycursive[idx] = true;
        *hasrecursive = true;
    } else {
        raycursive[idx] = false;
    }

    framebuffer[idx] = resultColor; 
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
    int width = 1600;
    int height = 900;
    SFMLTexture SFTex(width, height);
    
    int threads = 256;
    int blocks = (width * height + threads - 1) / threads;

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
    bool *d_raycursive; // Pun intended
    bool *d_hasrecursive;
    cudaMalloc(&d_raycursive, width * height * sizeof(bool));
    cudaMalloc(&d_hasrecursive, sizeof(bool));

    // Set all to true to kickstart the first iteration
    resetRecursive<<<blocks, threads>>>(d_raycursive, width, height);
    cudaDeviceSynchronize();

    // Set the hasrecursive true
    cudaMemcpy(d_hasrecursive, new bool(true), sizeof(bool), cudaMemcpyHostToDevice);

    // Creating some test triangles
    
    int tc = 0;
    Triangle triangles[11];
    // Postive Z
    triangles[tc].v0 = Vec3f(-10, -10, 50);
    triangles[tc].v1 = Vec3f(10, -10, 50);
    triangles[tc].v2 = Vec3f(0, 10, 50);
    triangles[tc].c1 = Vec3f(1, 0, 0);
    triangles[tc].c2 = Vec3f(1, 1, 1);
    triangles[tc].c3 = Vec3f(0, 0, 1);
    triangles[tc].uniformNormal(Vec3f(0, 0, -1));

    // Positive X
    triangles[++tc].v0 = Vec3f(50, -10, -10);
    triangles[tc].v1 = Vec3f(50, -10, 10);
    triangles[tc].v2 = Vec3f(50, 10, 0);
    triangles[tc].c1 = Vec3f(0, 1, 0);
    triangles[tc].c2 = Vec3f(0, 0, 1);
    triangles[tc].c3 = Vec3f(1, 0, 0);
    triangles[tc].uniformNormal(Vec3f(-1, 0, 0));

    // Negative X
    triangles[++tc].v0 = Vec3f(-50, -10, -10);
    triangles[tc].v1 = Vec3f(-50, -10, 10);
    triangles[tc].v2 = Vec3f(-50, 10, 0);
    triangles[tc].c1 = Vec3f(0, 1, 0);
    triangles[tc].c2 = Vec3f(0, 0, 1);
    triangles[tc].c3 = Vec3f(1,  0, 0);
    triangles[tc].uniformNormal(Vec3f(1, 0, 0));

    int mrWidth = 100;
    int mrHeight = 50;
    int mrDepth = 100;

    int wallWidth = 200;
    int wallHeight = 100;

    // Negative Z Mirror
    triangles[++tc].v0 = Vec3f(-mrWidth, -mrHeight, -mrDepth);
    triangles[tc].v1 = Vec3f(mrWidth, -mrHeight, -mrDepth);
    triangles[tc].v2 = Vec3f(mrWidth, mrHeight, -mrDepth);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));
    triangles[tc].reflect = true;
    triangles[++tc].v0 = Vec3f(-mrWidth, -mrHeight, -mrDepth);
    triangles[tc].v1 = Vec3f(mrWidth, mrHeight, -mrDepth);
    triangles[tc].v2 = Vec3f(-mrWidth, mrHeight, -mrDepth);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));
    triangles[tc].reflect = true;

    // Negative Z Wall
    triangles[++tc].v0 = Vec3f(-wallWidth, -wallHeight, -mrDepth - .1);
    triangles[tc].v1 = Vec3f(wallWidth, -wallHeight, -mrDepth - .1);
    triangles[tc].v2 = Vec3f(wallWidth, wallHeight, -mrDepth - .1);
    triangles[tc].c1 = Vec3f(0.6, 1, 0.6);
    triangles[tc].c2 = Vec3f(0.6, 0.6, 1);
    triangles[tc].c3 = Vec3f(1, 0.6, 0.6);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));
    triangles[++tc].v0 = Vec3f(-wallWidth, -wallHeight, -mrDepth - .1);
    triangles[tc].v1 = Vec3f(wallWidth, wallHeight, -mrDepth - .1);
    triangles[tc].v2 = Vec3f(-wallWidth, wallHeight, -mrDepth - .1);
    triangles[tc].c1 = Vec3f(0.6, 1, 0.6);
    triangles[tc].c2 = Vec3f(1, 0.6, 0.6);
    triangles[tc].c3 = Vec3f(0.6, 0.6, 1);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));

    // Positive Z Mirror
    triangles[++tc].v0 = Vec3f(-mrWidth, -mrHeight, mrDepth);
    triangles[tc].v1 = Vec3f(mrWidth, -mrHeight, mrDepth);
    triangles[tc].v2 = Vec3f(mrWidth, mrHeight, mrDepth);
    triangles[tc].uniformNormal(Vec3f(0, 0, -1));
    triangles[tc].reflect = true;
    triangles[tc].display = false;
    triangles[++tc].v0 = Vec3f(-mrWidth, -mrHeight, mrDepth);
    triangles[tc].v1 = Vec3f(mrWidth, mrHeight, mrDepth);
    triangles[tc].v2 = Vec3f(-mrWidth, mrHeight, mrDepth);
    triangles[tc].uniformNormal(Vec3f(0, 0, -1));
    triangles[tc].reflect = true;
    triangles[tc].display = false;

    // Positive Z Wall
    triangles[++tc].v0 = Vec3f(-wallWidth, -wallHeight, mrDepth + .1);
    triangles[tc].v1 = Vec3f(wallWidth, -wallHeight, mrDepth + .1);
    triangles[tc].v2 = Vec3f(wallWidth, wallHeight, mrDepth + .1);
    triangles[tc].c1 = Vec3f(1, 0.6, 0.6);;
    triangles[tc].c2 = Vec3f(0.6, 1, 0.6);;
    triangles[tc].c3 = Vec3f(0.6, 0.6, 1);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));
    triangles[++tc].v0 = Vec3f(-wallWidth, -wallHeight, mrDepth + .1);
    triangles[tc].v1 = Vec3f(wallWidth, wallHeight, mrDepth + .1);
    triangles[tc].v2 = Vec3f(-wallWidth, wallHeight, mrDepth + .1);
    triangles[tc].c1 = Vec3f(1, 0.6, 0.6);
    triangles[tc].c2 = Vec3f(0.6, 0.6, 1);
    triangles[tc].c3 = Vec3f(0.6, 1, 0.6);
    triangles[tc].uniformNormal(Vec3f(0, 0, 1));

    // Copy to device
    Triangle *d_triangles;
    cudaMalloc(&d_triangles, ++tc * sizeof(Triangle));
    cudaMemcpy(d_triangles, triangles, tc * sizeof(Triangle), cudaMemcpyHostToDevice);   

    // Create window
    sf::RenderWindow window(sf::VideoMode(width, height), "AsczEngine");
    window.setMouseCursorVisible(!CAMERA.focus);
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

        // Recursive ray tracing
        // Set all to true to kickstart the first iteration
        resetRecursive<<<blocks, threads>>>(d_raycursive, width, height);   
        cudaDeviceSynchronize();

        bool *hasrecursive = new bool(true);
        int recursionCount = -1;
        while (*hasrecursive) {
            *hasrecursive = false;
            cudaMemcpy(d_hasrecursive, hasrecursive, sizeof(bool), cudaMemcpyHostToDevice); 

            // Cast rays
            castRays<<<blocks, threads>>>(d_framebuffer, d_rays, d_raycursive, d_hasrecursive, d_triangles, width, height, tc);
            cudaDeviceSynchronize();
            
            // Copy hasrecursive to host
            cudaMemcpy(hasrecursive, d_hasrecursive, sizeof(bool), cudaMemcpyDeviceToHost);

            recursionCount++;

            if (recursionCount > 10) break; // Break if it's too much
        }

        // Log
        LOG.addLog("Welcome to AsczEngineRT v0", sf::Color::Green, 1);
        LOG.addLog("FPS: " + std::to_string(FPS.fps), sf::Color::Green);
        LOG.addLog("Recursion count: " + std::to_string(recursionCount), sf::Color::Red);

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