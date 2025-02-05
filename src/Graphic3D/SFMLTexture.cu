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
    blockNum = (width * height + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
}

void SFMLTexture::updateTexture(Flt3 *frmbuffer, int b_w, int b_h) {
    int bCount = (b_w * b_h + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;

    updateTextureKernel<<<bCount, THREADS_PER_BLOCK>>>(
        d_sfPixel, frmbuffer, b_w, b_h
    );
    cudaMemcpy(sfPixel, d_sfPixel, pixelCount * sizeof(sf::Uint8), cudaMemcpyDeviceToHost);
    texture.update(sfPixel);
}

// Kernel for updating the texture
__global__ void updateTextureKernel(
    sf::Uint8 *d_sfPixel, Flt3 *frmbuffer, int b_w, int b_h
) {
    int tIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tIdx >= b_w * b_h) return;

    int x = tIdx % b_w;
    int y = tIdx / b_w;
    int b_i = x + y * b_w;

    // Limit the color to 0-255 
    frmbuffer[b_i].x = fminf(fmaxf(frmbuffer[b_i].x, 0.0f), 1.0f);
    frmbuffer[b_i].y = fminf(fmaxf(frmbuffer[b_i].y, 0.0f), 1.0f);
    frmbuffer[b_i].z = fminf(fmaxf(frmbuffer[b_i].z, 0.0f), 1.0f);

    int p_i = b_i * 4;
    d_sfPixel[p_i + 0] = (sf::Uint8)(frmbuffer[b_i].x * 255);
    d_sfPixel[p_i + 1] = (sf::Uint8)(frmbuffer[b_i].y * 255);
    d_sfPixel[p_i + 2] = (sf::Uint8)(frmbuffer[b_i].z * 255);
    d_sfPixel[p_i + 3] = 255;
}