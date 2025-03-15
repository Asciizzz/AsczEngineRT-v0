#ifndef ASCZMESH_CUH
#define ASCZMESH_CUH

#include <vector>

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
    std::vector<float> vx, vy, vz;
    std::vector<float> nx, ny, nz;
    std::vector<float> tx, ty;

    std::vector<int> fv0, fv1, fv2; // Face vertices
    std::vector<int> fn0, fn1, fn2; // Face normals
    std::vector<int> ft0, ft1, ft2; // Face textures

    std::vector<int> fm; // Face materials
    std::vector<int> lsrc; // Light sources
    // std::vector<int> SOrF; // Sub-objects

    // // Object AABB
    float O_AB_min_x = INFINITY, O_AB_min_y = INFINITY, O_AB_min_z = INFINITY;
    float O_AB_max_x = -INFINITY, O_AB_max_y = -INFINITY, O_AB_max_z = -INFINITY;

    // // Sub-objects AABB
    // std::vector<float> SO_AB_min_x, SO_AB_min_y, SO_AB_min_z;
    // std::vector<float> SO_AB_max_x, SO_AB_max_y, SO_AB_max_z;
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

    // int oNum = 0; // Number of objects
    // std::vector<int>  OrSO = {0}; // Object references sub-objects
    // std::vector<int>  SOrF = {0}; // Sub-object references faces

    float GLB_min_x =  INFINITY, GLB_min_y =  INFINITY, GLB_min_z =  INFINITY;
    float GLB_max_x = -INFINITY, GLB_max_y = -INFINITY, GLB_max_z = -INFINITY;
    
    float *d_AB_min_x = nullptr, *d_AB_min_y = nullptr, *d_AB_min_z = nullptr;
    float *d_AB_max_x = nullptr, *d_AB_max_y = nullptr, *d_AB_max_z = nullptr;
    float *d_AB_cx = nullptr, *d_AB_cy = nullptr, *d_AB_cz = nullptr;
    float *AB_min_x, *AB_min_y, *AB_min_z;
    float *AB_max_x, *AB_max_y, *AB_max_z;
    float *AB_cx,    *AB_cy,    *AB_cz;

    void append(MeshStruct mesh);

    // Device memory
    float *d_vx = nullptr, *d_vy = nullptr, *d_vz = nullptr; int vNum = 0;
    float *d_nx = nullptr, *d_ny = nullptr, *d_nz = nullptr; int nNum = 0;
    float *d_tx = nullptr, *d_ty = nullptr;                  int tNum = 0;

    int *d_fv0 = nullptr, *d_fv1 = nullptr, *d_fv2 = nullptr;
    int *d_fn0 = nullptr, *d_fn1 = nullptr, *d_fn2 = nullptr;
    int *d_ft0 = nullptr, *d_ft1 = nullptr, *d_ft2 = nullptr;
    int *d_fm  = nullptr;                       int fNum = 0;

    // For multiple importance sampling
    int *d_lsrc = nullptr;                      int lNum = 0;

    void toDevice();
};

#endif