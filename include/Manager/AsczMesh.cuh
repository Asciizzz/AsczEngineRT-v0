#ifndef AsczMesh_CUH
#define AsczMesh_CUH

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
    Vecs3f v;
    Vecs2f t;
    Vecs3f n;

    Vecs3i fv;
    Vecs3i ft;
    Vecs3i fn;
    VecsI fm;

    VecsI SOrF; // Sub-objects
};

class AsczMesh {
public:
    // Host memory
    Vecs3f h_v;
    Vecs2f h_t;
    Vecs3f h_n;

    Vecs3i h_fv;
    Vecs3i h_ft;
    Vecs3i h_fn;
    VecsI  h_fm; // Face's material index
    Vecs3f h_fABmin; // Face's AABB min
    Vecs3f h_fABmax; // Face's AABB max
    Vecs3f h_fABcen; // Face's AABB center (not to be confused with face's center)

    VecsI OrSO = {0}; // Object references sub-objects
    VecsI SOrF = {0}; // Sub-object references faces


    Flt3 ABmin, ABmax;

    void appendMesh(MeshStruct mesh);
    void computeData();

    // Device memory
    Flt3 *d_v = nullptr; int vNum = 0;
    Flt2 *d_t = nullptr; int tNum = 0;
    Flt3 *d_n = nullptr; int nNum = 0;

    Int3 *d_fv = nullptr;
    Int3 *d_ft = nullptr;
    Int3 *d_fn = nullptr;
    int   *d_fm = nullptr;
    Flt3 *d_fABmin = nullptr;
    Flt3 *d_fABmax = nullptr;
    Flt3 *d_fABcen = nullptr;
    int fNum = 0;

    void freeDevice();
    void toDevice();
};

#endif