#include <SFMLTexture.cuh>

SFMLTexture::SFMLTexture(int width, int height) { resize(width, height); }

void SFMLTexture::free() {
    delete[] sfPixel;
    cudaFree(d_sfPixel);
}

void SFMLTexture::resize(int width, int height) {
    texture.create(width, height);
    sprite.setTexture(texture);

    // Allocate memory for the Pixel buffer
    sfPixel = new sf::Uint8[width * height * 4];
    cudaMalloc(&d_sfPixel, width * height * 4 * sizeof(sf::Uint8));

    pixelCount = width * height * 4;
    blockNum = (width * height + blockSize - 1) / blockSize;
}

void SFMLTexture::updateTexture(Vec3f *framebuffer, int b_w, int b_h, int p_s) {
    int bCount = (b_w * b_h + blockSize - 1) / blockSize;

    updateTextureKernel<<<bCount, blockSize>>>(
        d_sfPixel, framebuffer, b_w, b_h, p_s
    );
    cudaMemcpy(sfPixel, d_sfPixel, pixelCount * sizeof(sf::Uint8), cudaMemcpyDeviceToHost);
    texture.update(sfPixel);
}

// Kernel for updating the texture
__global__ void updateTextureKernel(
    sf::Uint8 *d_sfPixel, Vec3f *framebuffer, int b_w, int b_h, int p_s
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= b_w * b_h) return;

    int x = i % b_w;
    int y = i / b_w;
    int b_i = x + y * b_w;

    // Limit the color to 0-255 
    framebuffer[b_i].x = fminf(fmaxf(framebuffer[b_i].x, 0.0f), 1.0f);
    framebuffer[b_i].y = fminf(fmaxf(framebuffer[b_i].y, 0.0f), 1.0f);
    framebuffer[b_i].z = fminf(fmaxf(framebuffer[b_i].z, 0.0f), 1.0f);

    for (int i = 0; i < p_s; i++)
    for (int j = 0; j < p_s; j++) {
        int p_i = (x * p_s + i) + (y * p_s + j) * b_w * p_s;
        p_i *= 4;

        d_sfPixel[p_i + 0] = (sf::Uint8)(framebuffer[b_i].x * 255);
        d_sfPixel[p_i + 1] = (sf::Uint8)(framebuffer[b_i].y * 255);
        d_sfPixel[p_i + 2] = (sf::Uint8)(framebuffer[b_i].z * 255);
        d_sfPixel[p_i + 3] = 255;
    }
}