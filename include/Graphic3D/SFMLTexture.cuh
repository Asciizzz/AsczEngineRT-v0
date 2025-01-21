#ifndef SFMLTEXTURE_CUH
#define SFMLTEXTURE_CUH

#include <SFML/Graphics.hpp>
#include <Vector.cuh>

/*
Idea: since you cant just execute draw function in parallel, you can
instead create a texture, fill it with pixels IN PARALLEL, and then
draw the texture to the window. This way, you can utilize the GPU
to fill the pixels.
*/

class SFMLTexture {
public:
    sf::Texture texture;
    sf::Sprite sprite;

    // Allocate memory for the Pixel buffer
    sf::Uint8 *sfPixel;
    sf::Uint8 *d_sfPixel;
    int pixelCount;

    // Set kernel parameters
    int blockSize = 256;
    int blockNum;

    SFMLTexture(int width, int height);
    void free();
    void resize(int width, int height);

    void updateTexture(Vec3f *framebuffer, int b_w, int b_h);
};

__global__ void updateTextureKernel(
    sf::Uint8 *d_sfPixel, Vec3f *framebuffer, int b_w, int b_h
);

#endif