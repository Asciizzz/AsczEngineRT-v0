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

struct MeshStruct {
    Vec3f v;
    Vec2f t;
    Vec3f n;

    Vec3i fv;
    Vec3i ft;
    Vec3i fn;
    VecI  fm;

    VecI  SOrF; // Sub-objects
};

class AsczMesh {
public:
    // Host memory
    Vec3f h_v;
    Vec2f h_t;
    Vec3f h_n;

    Vec3i h_fv;
    Vec3i h_ft;
    Vec3i h_fn;
    VecI  h_fm; // Face's material index
    Vec3f h_ABmin; // Face's AABB min
    Vec3f h_ABmax; // Face's AABB max
    Vec3f h_ABcen; // Face's AABB center (not to be confused with face's center)

    VecI  OrSO = {0}; // Object references sub-objects
    VecI  SOrF = {0}; // Sub-object references faces

    void appendMesh(MeshStruct mesh);
    void computeData();

    // Device memory
    Flt3 *d_v = nullptr; int vNum = 0;
    Flt2 *d_t = nullptr; int tNum = 0;
    Flt3 *d_n = nullptr; int nNum = 0;

    Int3 *d_fv = nullptr;
    Int3 *d_ft = nullptr;
    Int3 *d_fn = nullptr;
    int  *d_fm = nullptr;
    Flt3 *d_ABmin = nullptr;
    Flt3 *d_ABmax = nullptr;
    Flt3 *d_ABcen = nullptr;
    int fNum = 0;

    void freeDevice();
    void toDevice();
};

#endif