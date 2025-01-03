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

__device__ float shadowMultiplier(Vec3f color) {
    // real color = base color * shadowMultiplier
    // Darker colors are less affected by shadows

    float a = 0.1;
    float b = 0.8;
    float k = 5.0;

    float avg = (color.x + color.y + color.z) / 3.0f;
    float multiplier = a + (b - a) / (1 + expf(-k * (avg - 0.5f)));
    return 1 - multiplier;
}

__global__ void clearFramebuffer(Vec3f *framebuffer, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    framebuffer[idx] = Vec3f(0, 0, 0);
}

__global__ void resetRecursive(bool *raycursive, int *recursionidx, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    raycursive[idx] = true; // To kickstart the first iteration
    recursionidx[idx] = idx;
}

__global__ void generateRays(Camera camera, Ray *rays, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    int x = idx % width;
    int y = idx / width;

    rays[y * width + x] = camera.castRay(x, y, width, height);
}

__global__ void castRays(
    Vec3f *framebuffer, Vec3f *vertexbuffer, Vec3f *normalbuffer,
    Ray *rays, bool *raycursive, int *recursionidx, bool *hasrecursive,
    Vec3f lightPos,
    Triangle *triangles, int width, int height, int triangleCount
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    if (!raycursive[idx]) return;

    Ray ray = rays[idx];

    int curTri = -1;
    float curZ = 1000000.0f;
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

        if (t > 0.00001 && t < curZ) {
            curZ = t;

            // Saving relevant data to avoid recalculating  
            curTri = i;
        }
    }

    if (curTri == recursionidx[idx] && curTri != -1) {
        raycursive[idx] = false;
        return;
    }

    bool recursive = triangles[curTri].reflect;

    Vec3f vertex = ray.origin + ray.direction * curZ;
    Vec3f bary = Vec3f::bary(vertex, triangles[curTri].v0, triangles[curTri].v1, triangles[curTri].v2);
    float u = bary.x;
    float v = bary.y;
    float w = bary.z;

    Vec3f normal = triangles[curTri].n0 * w
                + triangles[curTri].n1 * u
                + triangles[curTri].n2 * v;
    normal.norm();
    vertexbuffer[idx] = vertex;
    normalbuffer[idx] = normal;

    if (recursive) {
        // Set the recursive ray
        Ray recursiveRay;
        recursiveRay.direction = ray.reflect(normal);
        recursiveRay.origin = vertex + normal * 0.001f; // To avoid self-intersection
        rays[idx] = recursiveRay;

        raycursive[idx] = true;
        recursionidx[idx] = curTri;
        *hasrecursive = true;
    } else {
        raycursive[idx] = false;

        // Interpolate color
        Vec3f color = triangles[curTri].c0 * w
                    + triangles[curTri].c1 * u
                    + triangles[curTri].c2 * v;

        // Lighting (with ambient, diffuse, specular and shininess)
        Vec3f lightDir = lightPos - vertex;
        lightDir.norm();

        float ka = triangles[curTri].ambient;
        float kd = triangles[curTri].diffuse;
        float ks = triangles[curTri].specular;
        float shine = triangles[curTri].shininess;

        float ambient = ka;
        float diffuse = kd * fmaxf(0.0f, normal * lightDir);
        Vec3f reflectDir = lightDir - normal * (2.0f * (lightDir * normal));
        float specular = ks * pow(fmaxf(0.0f, ray.direction * reflectDir), shine);

        framebuffer[idx] = color * (ambient + diffuse + specular);
    }
}

__global__ void applyShadow(
    Vec3f *framebuffer, Vec3f *vertexbuffer, Vec3f *normalbuffer,
    Vec3f lightPos,
    Triangle *triangles, int width, int height, int triangleCount
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width * height) return;

    Ray ray;
    ray.origin = vertexbuffer[idx];
    ray.direction = lightPos - ray.origin;

    // Check if the line connecting the point and the light source intersects with any triangle
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

        if (t > 0.00001 && t < 1.0f) {
            // The point is in shadow
            framebuffer[idx] *= shadowMultiplier(framebuffer[idx]);
            break;
        }
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
    int width = 1600;
    int height = 900;
    SFMLTexture SFTex(width, height);

    int threads = 256;
    int blocks = (width * height + threads - 1) / threads;

    // Test camera
    Camera CAMERA;
    CAMERA.pos = Vec3f(0, 0, -10);
    CAMERA.rot = Vec3f(0, 0, 0);
    CAMERA.updateView();

    // Set up buffers
    Vec3f *d_framebuffer; // or colorbuffer
    Vec3f *d_vertexbuffer;
    Vec3f *d_normalbuffer;
    cudaMalloc(&d_framebuffer, width * height * sizeof(Vec3f));
    cudaMalloc(&d_vertexbuffer, width * height * sizeof(Vec3f));
    cudaMalloc(&d_normalbuffer, width * height * sizeof(Vec3f));

    // Set up rays
    Ray *d_rays;
    bool *d_raycursive; // Pun intended
    int *d_recursionidx; // The origin of the recursive ray
    bool *d_hasrecursive;
    cudaMalloc(&d_rays, width * height * sizeof(Ray));
    cudaMalloc(&d_raycursive, width * height * sizeof(bool));
    cudaMalloc(&d_recursionidx, width * height * sizeof(int));
    cudaMalloc(&d_hasrecursive, sizeof(bool));

    // Set the hasrecursive true
    cudaMemcpy(d_hasrecursive, new bool(true), sizeof(bool), cudaMemcpyHostToDevice);

    // Creating some test triangles
    std::vector<Triangle> room = Utils::readObjFile("test", "assets/Models/Shapes/Cube3.obj");
    #pragma omp parallel
    for (int i = 0; i < room.size(); i++) {
        int scaleFac = 40;
        room[i].v0.scale(Vec3f(), scaleFac);
        room[i].v1.scale(Vec3f(), scaleFac);
        room[i].v2.scale(Vec3f(), scaleFac);
    }

    std::vector<Triangle> shape1 = Utils::readObjFile("test", "assets/Models/Shapes/Test/test.obj");
    #pragma omp parallel
    for (Triangle &t : shape1) {
        t.reflect = true;
        // t.display = false;

        int scaleFac = 16;
        t.scale(Vec3f(), scaleFac);
        t.translate(Vec3f(0, 0, 39.8));
    }

    std::vector<Triangle> shape2 = Utils::readObjFile("test1", "assets/Models/Shapes/Test/test1.obj");
    #pragma omp parallel
    for (Triangle &t : shape2) {
        t.reflect = true;

        int scaleFac = 16;
        t.scale(Vec3f(), scaleFac);
        t.translate(Vec3f(0, 0, -39.8));
    }

    std::vector<Triangle> shape3 = Utils::readObjFile("test2", "assets/Models/Shapes/Test/test2.obj");
    #pragma omp parallel
    for (Triangle &t : shape3) {
        // t.reflect = true;

        int scaleFac = 5;
        t.scale(Vec3f(), scaleFac);
    }

    std::vector<Triangle> triangles = room;
    triangles.insert(triangles.end(), shape1.begin(), shape1.end());
    triangles.insert(triangles.end(), shape2.begin(), shape2.end());
    triangles.insert(triangles.end(), shape3.begin(), shape3.end());

    // Copy to device
    Triangle *d_triangles;
    int tc = triangles.size();
    cudaMalloc(&d_triangles, tc * sizeof(Triangle));
    cudaMemcpy(d_triangles, triangles.data(), tc * sizeof(Triangle), cudaMemcpyHostToDevice);

    // Test light   
    Vec3f lightSrc = Vec3f(0, 0, 0);

    // Create window
    sf::RenderWindow window(sf::VideoMode(width, height), "AsczEngine");
    sf::Mouse::setPosition(sf::Vector2i(width / 2, height / 2), window);
    window.setMouseCursorVisible(!CAMERA.focus);

    // Fun settings
    bool followLight = false;
    bool blackenScreen = false;

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

    // Black opaque rectangle
    sf::RectangleShape blackScreen(sf::Vector2f(width, height));
    blackScreen.setFillColor(sf::Color(0, 0, 0, 180));

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
                sf::Mouse::setPosition(sf::Vector2i(width / 2, height / 2), window);

                // Hide cursor
                window.setMouseCursorVisible(!CAMERA.focus);
            }

            // Press L to toggle light follow
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::L) {
                followLight = !followLight;
            }

            // Press B to toggle blacken screen
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::B) {
                blackenScreen = !blackenScreen;
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

        // Fun settings
        if (followLight) {
            lightSrc = CAMERA.pos;
        }

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
        resetRecursive<<<blocks, threads>>>(d_raycursive, d_recursionidx, width, height);
        cudaDeviceSynchronize();

        bool *hasrecursive = new bool(true);
        int recursionCount = -1;
        while (*hasrecursive) {
            *hasrecursive = false;
            cudaMemcpy(d_hasrecursive, hasrecursive, sizeof(bool), cudaMemcpyHostToDevice); 

            // Cast rays
            castRays<<<blocks, threads>>>(
                d_framebuffer, d_vertexbuffer, d_normalbuffer,
                d_rays, d_raycursive, d_recursionidx, d_hasrecursive,
                lightSrc,
                d_triangles, width, height, tc);
            cudaDeviceSynchronize();

            // Copy hasrecursive to host
            cudaMemcpy(hasrecursive, d_hasrecursive, sizeof(bool), cudaMemcpyDeviceToHost);

            recursionCount++;

            if (recursionCount == 10) break; // Break if it's too much
        }

        // Apply shadow
        applyShadow<<<blocks, threads>>>(
            d_framebuffer, d_vertexbuffer, d_normalbuffer,
            lightSrc,
            d_triangles, width, height, tc);
        cudaDeviceSynchronize();

        // Update "texture"
        SFTex.updateTexture(d_framebuffer, width, height, 1);

        // Log
        LOG.addLog("Welcome to AsczEngineRT v0", sf::Color::Green, 1);
        LOG.addLog("FPS: " + std::to_string(FPS.fps), sf::Color::Green);
        LOG.addLog("Recursion count: " + std::to_string(recursionCount), sf::Color::Red);
        LOG.addLog("Triangles count: " + std::to_string(tc), sf::Color::Red);
        LOG.addLog(CAMERA.data(), sf::Color(160, 255, 160));
        // Print the color at the dead center
        int idx = SFTex.pixelCount / 2 + width * 2;
        sf::Uint8 px1 = SFTex.sfPixel[idx + 0];
        sf::Uint8 px2 = SFTex.sfPixel[idx + 1];
        sf::Uint8 px3 = SFTex.sfPixel[idx + 2];
        Vec3f color = Vec3f(px1, px2, px3);
        LOG.addLog("Color: "
            + std::to_string(color.x) + ", "
            + std::to_string(color.y) + ", "
            + std::to_string(color.z),
        sf::Color(255 - px1, 255 - px2, 255 - px3)); // Contrast color for better visibility

        // Draw to window
        window.clear(sf::Color::Black);
        window.draw(SFTex.sprite);
        // To see the log better
        if (blackenScreen) window.draw(blackScreen);
        LOG.drawLog(window);
        window.draw(crosshair1);
        window.draw(crosshair2);
        window.display();

        // Frame end
        FPS.endFrame();
    }

    return 0;
}