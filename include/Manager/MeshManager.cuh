#ifndef MESHMANAGER_CUH
#define MESHMANAGER_CUH

#include <Vector.cuh>
#define VectI std::vector<int>

/* OrSO and SOrF explanation:

The idea: .obj file can contain multiple o <object_name>, which we will called sub-objects.
To handle this, I have came up with a way to handle objects as well as their sub-objects.

== OrSO: Object reference sub-objects ==

- This array will store the ptr to the sub-objects
- Example: 3 objects: Human, Animal and Tree
    - Human contain 3 sub-objects: head, body, and legs
    - Animal contain 2 sub-objects: head and body
    - Tree contain 4 sub-objects: trunk, leaves, roots, and branches

    => O = {human, animal, tree}
    => SO = {head, body, legs, head, body, trunk, leaves, roots, branches}
    => OrSO = {0, 3, 5, 9}, OrSO[i] = OrSO[i-1] + number of sub-objects in O[i-1]

== SOrF: Sub-object reference faces ==

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
    Vecs3f v;
    Vecs2f t;
    Vecs3f n;

    Vecs3i fv;
    Vecs3i ft;
    Vecs3i fn;
    VectI fm;

    VectI SOrF; // Sub-object start index
};

class MeshManager {
public:
    // Host memory
    Vecs3f h_v;
    Vecs2f h_t;
    Vecs3f h_n;

    Vecs3i h_fv;
    Vecs3i h_ft;
    Vecs3i h_fn;
    VectI h_fm;

    VectI OrSO = {0}; // Object reference sub-objects
    VectI SOrF = {0}; // Sub-object reference faces

    Vecs3f h_fmin; // Face's AABB min
    Vecs3f h_fmax; // Face's AABB max

    Vec3f ABmin, ABmax;

    void appendMesh(MeshStruct mesh);
    void computeData();

    // Device memory
    Vec3f *d_v = nullptr; int vNum = 0;
    Vec2f *d_t = nullptr; int tNum = 0;
    Vec3f *d_n = nullptr; int nNum = 0;

    Vec3i *d_fv = nullptr;
    Vec3i *d_ft = nullptr;
    Vec3i *d_fn = nullptr;
    int *d_fm = nullptr;
    int fNum = 0;

    void freeDevice();
    void hostToDevice();
};

#endif