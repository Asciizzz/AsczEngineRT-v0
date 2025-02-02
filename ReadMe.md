# AsczEngineRT_v0: A very Skibidy 3D Ray Tracing Graphics Engine

**AsczEngineRT_v0** is built from scratch in C++ and uses only CUDA for parallel processing, oh and SFML for the goddamn window lol. Since you're here anyway, I suppose you can learn a thing or two on how and/or how not to create an engine. Suggestions are appreciated my fellow fanum taxers.

### How it worked

The idea behind ray tracing is simple: you trace ray.

But instead of tracing pretty-much-infinite-number of rays from *light* $\rightarrow$ *surface* $\rightarrow$ *camera* (like how light would irl), you trace a finite number of ray from the *camera* $\rightarrow$ *surface* $\rightarrow$ *light* and do the math in reverese. This may seem quite counter-intuitive, but the performace gain is GYATTDAMN massive since $x\rightarrow \infin$ of rays from the light doesn't even reach the camera in the first place.

Ray tracing allow for graphically complex and graphically beutiful grapich, since rays can bounces around, which allow for stuff like reflection or refraction to be possible. To handle ray tracing, 2 methods were proposed (by the great me ofc):

- **Recursive Ray Tracing**: the idea is the result of the next ray will influence the result of the current ray, so you trace the next ray and the next ray and the next ray... until you reach the limit of the recursion. This method is simple to implement, but stack overflow is a b*tch, so it's not recommended.

- **Weight Based Ray Tracing**: this is similar to Monte Carlo's path tracing, but deterministic instead of stochastic. Each ray has a weight, the next ray will influence the weight of the current ray, but the total weight of every ray will always be the same (sum = 1).
  - Example: 
    - Primary ray `weight = 1.0` $\rightarrow$ Hit a red surface with a `reflective = 0.5` $\rightarrow$ Primary ray `weight = 1.0 * 0.5 = 0.5` + New ray 1 `weight = 1.0 * 0.5 = 0.5`.
    - New ray 1 hit a blue surface with a `reflective = 0.5` $\rightarrow$ New ray 1 `weight = 0.5 * 0.5 = 0.25` + New ray 2 `weight = 0.5 * 0.5 = 0.25`.
    - --- 
    - $\Rightarrow$ **`Result Color`** `=` **`Blue * 0.5`** *`(from primary ray)`* `+` **`Red * 0.25`** *`(from new ray 1)`* `+` **`Blue * 0.25`** *`(from new ray 2)`* `=` **`Something idk do the math yourself`**.
    - $\Rightarrow$ **`Result Weight`** `=` `0.5 + 0.25 + 0.25` `=` **`1.0`**.

### How to Use

Just run the `AsczEngineRT_v0.exe` file, easy as that.

### Contribute

Whether you have ideas, suggestions, techniques, or just want to discuss rendering techniques, contributions are welcome.


# Ignore

- Geometry Processing: LOD, culling, vertex cache optimization.
- Rasterization: Early Z-culling, triangle rasterization optimizations, tile-based rendering.
- Fragment Shading: Early tests (Z, alpha), shader program optimizations, MSAA.
- Post-Processing: Deferred shading, screen-space effects.
- General: Parallelism, memory management.
- BVH traversal: Stackless traversal, Surface Area Herobrine.
- Ray tracing: recursive vs iterative (recommended).
- Spaghetti: onion, garlic powder, tomato, meatball...

![](assets/Textures/Seia.png)