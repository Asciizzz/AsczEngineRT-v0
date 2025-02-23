#include <Utility.cuh>

void Utils::appendObj(
    AsczMesh &MeshMgr, AsczMat &MatMgr, AsczTxtr &TxtrMgr,
    const char *objPath, short placement, float scale, short fIdxBased 
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return;

    Vec3f mv;
    Vec2f mt;
    Vec3f mn;

    VecGeom mgeom;
    VecI mlSrc;

    VecI mSOrF;

    AABB mO_AB; // Object AABB
    VecAB mSO_AB; // Sub-objects AABB

    int matIdx = 0;
    bool matEms = false;
    std::unordered_map<std::string, int> matMap;

    std::string path(objPath);

    std::string line;
    while (std::getline(file, line)) {
        if (line.size() == 0 || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string type; ss >> type;

        // The datatype has been sorted by their frequency
        /* Ranking:
            * v: 1 - most frequent
            * f: 2 - very frequent
            * vt: 3 - quite frequent
            * vn: 4 - quite frequent
            * o: 5 - not so frequent
            * usemtl: 6 - not so frequent
            * mtllib: 7 - only once
        */

        if (type == "v") {
            Flt3 v; ss >> v.x >> v.y >> v.z;
            v.scale(Flt3(), scale);
            mv.push_back(v);
        }

        else if (type == "f") {
            VecI vs, ts, ns;
            while (ss.good()) {
                std::string vtn; ss >> vtn;
                std::stringstream ss2(vtn);

                int v, t, n;

                // Read vertex index
                ss2 >> v;

                // Check for texture index (skip if missing)
                if (ss2.peek() == '/') {
                    ss2.ignore(1); // Ignore the first '/'
                    if (ss2.peek() != '/') {
                        ss2 >> t; // Read texture index if present
                    } else {
                        t = fIdxBased - 1; // No texture index provided
                    }
                } else {
                    t = fIdxBased - 1; // No slashes, so no texture coordinate
                }

                // Check for normal index
                if (ss2.peek() == '/') {
                    ss2.ignore(1); // Ignore the second '/'
                    ss2 >> n; // Read normal index
                } else {
                    n = fIdxBased - 1 ; // No normal index provided
                }

                vs.push_back(v - fIdxBased);
                ts.push_back(t - fIdxBased);
                ns.push_back(n - fIdxBased);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; ++i) {
                if (matEms) mlSrc.push_back(mgeom.size());

                mgeom.push_back(AzGeom(
                    Int3(vs[0], vs[i], vs[i + 1]),
                    Int3(ts[0], ts[i], ts[i + 1]),
                    Int3(ns[0], ns[i], ns[i + 1]),
                matIdx));
            }

            // Expand the AABB  
            for (int i = 0; i < vs.size(); ++i) {
                mO_AB.expand(mv[vs[i]]);
                mSO_AB.back().expand(mv[vs[i]]);
            }
        }

        else if (type == "vt") {
            Flt2 t; ss >> t.x >> t.y;
            mt.push_back(t);
        }

        else if (type == "vn") {
            Flt3 n; ss >> n.x >> n.y >> n.z;
            mn.push_back(n.norm());
        }

        else if (type == "o") {
            mSOrF.push_back(mgeom.size());
            mSO_AB.push_back(AABB());
        }

        else if (type == "usemtl") {
            std::string matName;
            ss >> matName;

            matIdx = matMap[matName];

            // Check if the material is emissive
            matEms = !MatMgr.h_mtls[matIdx].Ems.isZero();
        }

        else if (type == "mtllib" || type == "azmlib") {
            std::string mtlPath;
            ss >> mtlPath;

            std::string mtlDir = path.substr(0, path.find_last_of("/\\") + 1);
            std::ifstream mtlFile(mtlDir + mtlPath);
            if (!mtlFile.is_open()) continue;

            std::string mtlLine;
            while (std::getline(mtlFile, mtlLine)) {
                if (mtlLine.size() == 0 || mtlLine[0] == '#') continue;

                std::stringstream mtlSS(mtlLine);
                std::string mtlType;
                mtlSS >> mtlType;

                if (mtlType == "newmtl" || mtlType == "AzMtl") {
                    matIdx = MatMgr.appendMaterial(AzMtl());
                    std::string matName; mtlSS >> matName;
                    matMap[matName] = matIdx;
                }
                // Albedo
                else if (mtlType == "Kd" || mtlType == "Alb") {
                    Flt3 Kd; mtlSS >> Kd.x >> Kd.y >> Kd.z;
                    MatMgr.h_mtls[matIdx].Alb = Kd;
                }
                // Albedo map   
                else if (mtlType == "map_Kd" || mtlType == "AlbMap") {
                    std::string txtrPath; mtlSS >> txtrPath;

                    MatMgr.h_mtls[matIdx].AlbMap = TxtrMgr.appendTexture(
                        (mtlDir + txtrPath).c_str()
                    );
                }
                // Roughness
                else if (mtlType == "Ns" || mtlType == "Rough") {
                    float Ns; mtlSS >> Ns;
                    MatMgr.h_mtls[matIdx].Rough = 1 - Ns / 1000;
                }
                // Metallic
                else if (mtlType == "Ks" || mtlType == "Metal") {
                    Flt3 Ks; mtlSS >> Ks.x >> Ks.y >> Ks.z;
                    MatMgr.h_mtls[matIdx].Metal = Ks.x;
                }
                // Transmission
                else if (mtlType == "Tr") {
                    float Tr; mtlSS >> Tr;
                    MatMgr.h_mtls[matIdx].Tr = Tr;
                }
                else if (mtlType == "d") { // The opposite of Tr
                    float Tr; mtlSS >> Tr;
                    MatMgr.h_mtls[matIdx].Tr = 1 - Tr;
                }
                // Index of refraction
                else if (mtlType == "Ni" || mtlType == "Ior") {
                    float Ni; mtlSS >> Ni;
                    MatMgr.h_mtls[matIdx].Ior = Ni;
                }
                // Emission
                else if (mtlType == "Ke" || mtlType == "Ems") {
                    Flt3 Ke; mtlSS >> Ke.x >> Ke.y >> Ke.z;
                    MatMgr.h_mtls[matIdx].Ems = Ke;
                }
            }
        }

        // STUFF THAT DO NOT EXIST IN A TYPICAL .OBJ FILE
        else if (type == "sph") {
            int c; float r; ss >> c >> r;
            mgeom.push_back(AzGeom(c - fIdxBased, r, matIdx));

            // Get the AABB of the sphere
            Flt3 sc = mv[c - fIdxBased];
            AABB ab(sc - r, sc + r);

            mO_AB.expand(ab);
            mSO_AB.back().expand(ab);
        }
    }
    mSOrF.push_back(mgeom.size());
    mSOrF.erase(mSOrF.begin());

    // ---------------------------------------------------------

    Flt3 shift;
    if (placement > 0) {
        shift.x = (mO_AB.min.x + mO_AB.max.x) / 2;
        shift.z = (mO_AB.min.z + mO_AB.max.z) / 2;
    }
    if (placement == 1) {
        shift.y = (mO_AB.min.y + mO_AB.max.y) / 2;
    }
    else if (placement == 2) {
        shift.y = mO_AB.min.y;
    }

    #pragma omp parallel for
    for (size_t i = 0; i < mv.size(); ++i) {
        mv[i] -= shift;
    }

    // Shift the AABBs
    mO_AB.min -= shift;
    mO_AB.max -= shift;

    for (AABB &ab : mSO_AB) {
        ab.min -= shift;
        ab.max -= shift;
    }

    // ---------------------------------------------------------

    MeshStruct mesh;
    mesh.v = mv;
    mesh.t = mt;
    mesh.n = mn;
    mesh.geom = mgeom;
    mesh.SOrF = mSOrF;
    mesh.O_AB = mO_AB;
    mesh.SO_AB = mSO_AB;
    mesh.lSrc = mlSrc;

    MeshMgr.appendMesh(mesh);
}