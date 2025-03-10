#include <Utility.cuh>

void Utils::appendObj(
    AsczMesh &MeshMgr, AsczMat &MatMgr, AsczTxtr &TxtrMgr,
    const char *objPath, short placement, float scale, float yaw, Flt3 trans
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return;

    std::vector<Flt3> mv;
    std::vector<Flt2> mt;
    std::vector<Flt3> mn;

    std::vector<Int3> mfv;
    std::vector<Int3> mft;
    std::vector<Int3> mfn;
    std::vector<int> mfm;
    std::vector<int> mlsrc;

    std::vector<int> mSOrF;

    AABB mO_AB; // Object AABB
    std::vector<AABB> mSO_AB; // Sub-objects AABB

    int matIdx = 0;
    bool matIsLight = false;
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
            v.scale(0, scale);

            // Rotate the vertex
            v.x = v.x * cos(yaw) - v.z * sin(yaw);
            v.z = v.x * sin(yaw) + v.z * cos(yaw);

            v += trans;

            mv.push_back(v);
            continue;
        }
        else if (type == "f") {
            std::vector<int> vs, ts, ns;
            while (ss.good()) {
                std::string vtn; ss >> vtn;
                std::stringstream ss2(vtn);
                int v, t, n;

                // Read vertex index
                ss2 >> v;

                // Check for texture index (skip if missing)
                if (ss2.peek() == '/') {
                    ss2.ignore(1); // Ignore the first '/'
                    if (ss2.peek() != '/') ss2 >> t; else t = 0;
                } else t = 0;

                // Check for normal index (skip if missing)
                if (ss2.peek() == '/') {
                    ss2.ignore(1); ss2 >> n; // Read normal index
                } else n = 0;

                vs.push_back(v < 0 ? mv.size() + v : v - 1);
                ts.push_back(t < 0 ? mt.size() + t : t - 1);
                ns.push_back(n < 0 ? mn.size() + n : n - 1);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; ++i) {
                mfv.push_back(Int3(vs[0], vs[i], vs[i + 1]));
                mft.push_back(Int3(ts[0], ts[i], ts[i + 1]));
                mfn.push_back(Int3(ns[0], ns[i], ns[i + 1]));

                mfm.push_back(matIdx);

                if (matIsLight) mlsrc.push_back(mfv.size() - 1);
            }

            // Expand the AABB  
            for (int i = 0; i < vs.size(); ++i) {
                mO_AB.expand(mv[vs[i]]);
                mSO_AB.back().expand(mv[vs[i]]);
            }
            continue;
        }

        else if (type == "vt") {
            Flt2 t; ss >> t.x >> t.y;
            mt.push_back(t);continue;
        }

        else if (type == "vn") {
            Flt3 n; ss >> n.x >> n.y >> n.z;

            // Rotate the normal
            if (yaw) {
                Flt3 n2 = n;
                n.x = n2.x * cos(yaw) - n2.z * sin(yaw);
                n.z = n2.x * sin(yaw) + n2.z * cos(yaw);
            }

            mn.push_back(n.norm());continue;
        }

        else if (type == "o") {
            mSOrF.push_back(mfv.size());
            mSO_AB.push_back(AABB());continue;
        }

        else if (type == "usemtl" || type == "AzMtl") {
            std::string matName;
            ss >> matName;

            matIdx = matMap[matName];
            matIsLight = MatMgr.h_mtls[matIdx].Ems_i;
            continue;
        }

        else if (type == "mtllib" || type == "AzmLib") {
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
                    std::string matName; mtlSS >> matName;
                    std::string matPath = mtlDir;
                    matIdx = MatMgr.append(AzMtl(),
                        std::wstring(matName.begin(), matName.end()), 
                        std::wstring(mtlDir.begin(), mtlDir.end())
                    );
                    matMap[matName] = matIdx;
                }
                // Albedo
                else if (mtlType == "Kd" || mtlType == "Alb") {
                    // Flt3 Kd; mtlSS >> Kd.x >> Kd.y >> Kd.z;
                    float alb_r, alb_g, alb_b;
                    mtlSS >> alb_r >> alb_g >> alb_b;

                    MatMgr.h_mtls[matIdx].Alb_r = alb_r;
                    MatMgr.h_mtls[matIdx].Alb_g = alb_g;
                    MatMgr.h_mtls[matIdx].Alb_b = alb_b;
                }
                // Albedo map
                else if (mtlType == "map_Kd" || mtlType == "AlbMap") {
                    std::string txtrPath; mtlSS >> txtrPath;

                    MatMgr.h_mtls[matIdx].AlbMap = TxtrMgr.appendTexture(
                        (mtlDir + txtrPath).c_str()
                    );
                }
                // Roughness
                else if (mtlType == "Rough") {
                    float Rough; mtlSS >> Rough;
                    MatMgr.h_mtls[matIdx].Rough = Rough;
                }
                else if (mtlType == "Ns") { // Outdated
                    float Ns; mtlSS >> Ns;
                    MatMgr.h_mtls[matIdx].Rough = 1.0f - Ns / 1000.0f;
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
                    float Ior; mtlSS >> Ior;
                    MatMgr.h_mtls[matIdx].Ior = Ior;
                }
                // Emission
                else if (mtlType == "Ke" || mtlType == "Ems") {
                    // Flt4 Ems; mtlSS >> Ems.x >> Ems.y >> Ems.z >> Ems.w;
                    float Ems_r, Ems_g, Ems_b, Ems_i;
                    mtlSS >> Ems_r >> Ems_g >> Ems_b >> Ems_i;
                    Ems_i = Ems_i + !Ems_i; // In case the intensity is 0
                    MatMgr.h_mtls[matIdx].Ems_r = Ems_r;
                    MatMgr.h_mtls[matIdx].Ems_g = Ems_g;
                    MatMgr.h_mtls[matIdx].Ems_b = Ems_b;
                    MatMgr.h_mtls[matIdx].Ems_i = Ems_i;
                }

                // DEBUG VALUES
                else if (mtlType == "NoShade") {
                    MatMgr.h_mtls[matIdx].NoShade = true;
                }
            }
            continue;
        }
    }
    mSOrF.push_back(mfv.size());
    mSOrF.erase(mSOrF.begin());

    // ---------------------------------------------------------

    Flt3 shift;
    // In the middle of the y-axis
    if (placement == 1) {
        shift.y = (mO_AB.min.y + mO_AB.max.y) / 2;
    }
    // On the floor
    else if (placement == 2) {
        shift.y = mO_AB.min.y;
    }
    // On the floor and in the dead center
    else if (placement == 3) {
        shift.y = mO_AB.min.y;
        shift.x = (mO_AB.min.x + mO_AB.max.x) / 2;
        shift.z = (mO_AB.min.z + mO_AB.max.z) / 2;
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
    mesh.fv = mfv;
    mesh.ft = mft;
    mesh.fn = mfn;
    mesh.fm = mfm;
    mesh.lsrc = mlsrc;
    mesh.SOrF = mSOrF;
    mesh.O_AB = mO_AB;
    mesh.SO_AB = mSO_AB;

    MeshMgr.append(mesh);
}