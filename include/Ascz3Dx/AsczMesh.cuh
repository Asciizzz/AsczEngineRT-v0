#ifndef ASCZMESH_CUH
#define ASCZMESH_CUH

#include <Geom.cuh>

/* OrSO and SOrF explanation:

The idea: .obj file can contain multiple o <object_name>, which we will called sub-objects.
To handle this, I have came up with a way to handle objects as well as their sub-objects.

== OrSO: Object references sub-objects ==

- This array will store the ptr to the sub-objects
- Example: 3 objects: Human, Animal and Tree
    - Human contain 3 sub-objects: head, body, and legs
    - Animal contain 2 sub-objects: head and body
    - Tree contain 4 sub-objects: trunk, leaves, roots, and branches

    => O = {human, animal, tree}
    => SO = {head, body, legs, head, body, trunk, leaves, roots, branches}
    => OrSO = {0, 3, 5, 9}, OrSO[i] = OrSO[i-1] + number of sub-objects in O[i-1]

== SOrF: Sub-object references faces ==

- This array will store the ptr to the faces
- Example: Object Human contain 3 sub-objects: head, body, and legs
    - Head contain 2 faces
    - Body contain 3 faces
    - Legs contain 4 faces

    => SO = {head, body, legs}
    => SOrF = {0, 2, 5, 9}, SOrF[i] = SOrF[i-1] + number of faces in SO[i-1]

- To handle multiple objects, SOrF will offset itself accordingly
- Example: 2 objects: Human and Animal
    - Human contain 3 sub-objects: head (2 faces), body (3 faces), and legs (4 faces)
    - Animal contain 2 sub-objects: head (2 faces) and body (3 faces)

    => O = {human, animal}
    => SO = {head, body, legs, head, body}
    => OrSO = {0, 3, 5}
    => SOrF = Human{0, 2, 5, 9} + Animal{11, 14} = {0, 2, 5, 9, 11, 14}
*/

#define VecAB std::vector<AABB>

struct AABB {
    Flt3 min, max;

    __host__ __device__ AABB(
        Flt3 min = Flt3(INFINITY), Flt3 max = Flt3(-INFINITY)
    ) : min(min), max(max) {}

    __host__ __device__ void expandMin(const Flt3 &v) {
        min.x = fminf(min.x, v.x);
        min.y = fminf(min.y, v.y);
        min.z = fminf(min.z, v.z);
    }
    __host__ __device__ void expandMax(const Flt3 &v) {
        max.x = fmaxf(max.x, v.x);
        max.y = fmaxf(max.y, v.y);
        max.z = fmaxf(max.z, v.z);
    }
    __host__ __device__ void expand(const Flt3 &v) {
        expandMin(v);
        expandMax(v);
    }
    __host__ __device__ void expand(const AABB &ab) {
        expandMin(ab.min);
        expandMax(ab.max);
    }
    __host__ __device__ float getSA() const {
        Flt3 size = max - min;
        return size.x * size.y + size.y * size.z + size.z * size.x;
    }

    __host__ __device__ Flt3 cent() const {
        return (min + max) * 0.5f;
    }
};

struct MeshStruct {
    Vec3f v;
    Vec2f t;
    Vec3f n;

    VecGeom geom;

    VecI  SOrF; // Sub-objects

    AABB O_AB; // Object AABB
    VecAB SO_AB; // Sub-objects AABB
};

class AsczMesh {
public:
    // Host memory
    Vec3f h_v;
    Vec2f h_t;
    Vec3f h_n;

    int appendVrtx(Flt3 v);
    int appendTxtr(Flt2 t);
    int appendNrml(Flt3 n);

    // Extra geometry data for exclusive use (skybox, etc.)
    VecGeom h_geomEx;

    // The main geometry data
    VecGeom h_geom;

    int oNum = 0; // Number of objects
    VecI  OrSO = {0}; // Object references sub-objects
    VecI  SOrF = {0}; // Sub-object references faces

    AABB GlbAB; // Global AABB
    VecAB O_AB; // Objects AABB
    VecAB SO_AB; // Sub-objects AABB
    VecAB G_AB; // Geoms AABB

    void appendMesh(MeshStruct mesh);

    // Device memory
    Flt3 *d_v = nullptr; int vNum = 0;
    Flt2 *d_t = nullptr; int tNum = 0;
    Flt3 *d_n = nullptr; int nNum = 0;

    AzGeom *d_geom = nullptr;
    int gNum = 0;

    AzGeom *d_geomEx = nullptr;
    int gNumEx = 0;

    void freeDevice();
    void toDevice();
};

#endif