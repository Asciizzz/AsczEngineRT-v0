#ifndef ASCZMESH_CUH
#define ASCZMESH_CUH

#include <Vector.cuh>

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
    std::vector<Flt3> v;
    std::vector<Flt3> n;
    std::vector<Flt2> t;

    std::vector<Int3> fv;
    std::vector<Int3> fn;
    std::vector<Int3> ft;

    std::vector<int>  fm; // Face materials
    std::vector<int>  lsrc; // Light sources
    std::vector<int>  SOrF; // Sub-objects

    AABB O_AB; // Object AABB
    std::vector<AABB> SO_AB; // Sub-objects AABB
};

class AsczMesh {
public:
    ~AsczMesh();

    // SoA for better memory access
    std::vector<float> h_vx = {0.0f}, h_vy = {0.0f}, h_vz = {0.0f};
    std::vector<float> h_nx = {0.0f}, h_ny = {0.0f}, h_nz = {0.0f};
    std::vector<float> h_tx = {0.0f}, h_ty = {0.0f};

    // Geometry
    // VecGeom h_geom;
    std::vector<int> h_fv0, h_fv1, h_fv2; // Face vertices index
    std::vector<int> h_fn0, h_fn1, h_fn2; // Face normals index
    std::vector<int> h_ft0, h_ft1, h_ft2; // Face textures index
    std::vector<int> h_fm; // Face materials index
    std::vector<int> h_lsrc; // Light sources index

    int oNum = 0; // Number of objects
    std::vector<int>  OrSO = {0}; // Object references sub-objects
    std::vector<int>  SOrF = {0}; // Sub-object references faces

    AABB GlbAB; // Global AABB
    std::vector<AABB> O_AB; // Objects AABB
    std::vector<AABB> SO_AB; // Sub-objects AABB
    std::vector<AABB> G_AB; // Geoms AABB

    void append(MeshStruct mesh);

    // Device memory
    float *d_vx = nullptr, *d_vy = nullptr, *d_vz = nullptr; int vNum = 0;
    float *d_nx = nullptr, *d_ny = nullptr, *d_nz = nullptr; int nNum = 0;
    float *d_tx = nullptr, *d_ty = nullptr;                  int tNum = 0;

    int *d_fv0 = nullptr, *d_fv1 = nullptr, *d_fv2 = nullptr;
    int *d_fn0 = nullptr, *d_fn1 = nullptr, *d_fn2 = nullptr;
    int *d_ft0 = nullptr, *d_ft1 = nullptr, *d_ft2 = nullptr;
    int *d_fm  = nullptr;                       int gNum = 0;

    // For multiple importance sampling
    int *d_lsrc = nullptr;                      int lNum = 0;

    void toDevice();
};

#endif