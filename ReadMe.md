# AsczEngineRT_v0: Building the Foundations of 3D Rendering

**AsczEngineRT_v0** is the starting point of my journey toward creating a powerful and versatile 3D graphics engine. The focus of this version is on rendering static images with high quality, laying down techniques and parameters that will eventually enable real-time rendering in future iterations.

This engine is written in C++, with CUDA parallel processing capabilities. It’s a project from, well, even lower than scratch. Avoiding pre-built engines or graphics libraries (entirely), to gain a deeper understanding of 3D rendering principles.

### Why Static Rendering First?

Rendering a still image allows full focus on accuracy and visual fidelity without worrying about the complexities of real-time performance and other mind numbing optimizations shenanigans. This approach ensures:

- A solid understanding of how rasterization and ray tracing work individually and together.
    - ###### Note: Modern engine tend to combine both techniques for optimal performance and visual quality. For rendering a static image, I could have just used ray tracing, but implementing rasterization allows the engine to scale to real-time rendering in the future.
- The ability to experiment with lighting, shadows, and materials in detail.
The flexibility to refine techniques before introducing the constraints of real-time rendering.
- For now, think of it as mastering the fundamentals before moving on to more dynamic challenges.

### Key Features of AsczEngineRT_v0

##### Hybrid Rendering:

- Rasterization for efficient geometry rendering.
- Ray tracing for realistic lighting effects such as shadows and reflections.

##### Lighting and Shading:

- Support for ambient, diffuse, and specular lighting using the Phong reflection model.
- Shadows calculated statically for added realism.
Focus on Visual Quality:

##### Accurate Material Representation:

- Full support for material properties defined in .mtl files, including diffuse, ambient, and specular colors.
- Integration of specular exponents and refractive indices for physically accurate light interactions.
- Support for texture maps (e.g., diffuse, specular, bump) to enhance realism.
- Built with the goal of achieving absolute photorealism (well as close as it can get I suppose), enabling materials to behave as they would in the real world.


##### What’s Next?

While v0 is all about rendering static frames, the ultimate aim for AsczEngineRT is to grow into a full-fledged real-time graphics engine, soon to be promoted to a real-time 3D game engine.

### How to Use

For the time being, there is no way to use this engine as a standalone tool. However, you can explore the code to understand how the rendering process works and experiment with different techniques.

### Philosophy

This engine is built not just as a tool but as a learning experience. Every feature is implemented from absolute scratch to deepen understanding of 3D rendering principles as well as real-time graphics programming. The current version is completely ignoring the later part of the statement, but it will get there eventually (hopefully).

### Contribute

Whether you have ideas, suggestions, techniques, or just want to discuss rendering techniques, contributions are welcome.



# Ignore

- Geometry Processing: LOD, culling, vertex cache optimization.
- Rasterization: Early Z-culling, triangle rasterization optimizations, tile-based rendering.
- Fragment Shading: Early tests (Z, alpha), shader program optimizations, MSAA.
- Post-Processing: Deferred shading, screen-space effects.
- General: Parallelism, memory management.
- BVH traversal: Stackless traversal.