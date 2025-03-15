#include <Utility.cuh>

void Utils::appendObj(
    AsczMesh &MeshMgr, AsczMat &MatMgr, AsczTxtr &TxtrMgr,
    const char *objPath, short placement, float scale, float yaw, Flt3 trans
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return;

    // std::vector<float> mvx, mvy, mvz;
    // std::vector<float> mnx, mny, mnz;
    // std::vector<float> mtx, mty;

    // std::vector<int> mfv0, mfv1, mfv2;
    // std::vector<int> mfn0, mfn1, mfn2;
    // std::vector<int> mft0, mft1, mft2;
    // std::vector<int> mfm;
    // std::vector<int> mlsrc;

    // std::vector<int> mSOrF;

    MeshStruct MS;

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
            float vx, vy, vz; ss >> vx >> vy >> vz;
            vx *= scale; vy *= scale; vz *= scale;

            float x = vx, z = vz;
            vx = x * cos(yaw) - z * sin(yaw);
            vz = x * sin(yaw) + z * cos(yaw);

            MS.vx.push_back(vx + trans.x);
            MS.vy.push_back(vy + trans.y);
            MS.vz.push_back(vz + trans.z);

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

                vs.push_back(v < 0 ? MS.vx.size() + v : v - 1);
                ts.push_back(t < 0 ? MS.tx.size() + t : t - 1);
                ns.push_back(n < 0 ? MS.nx.size() + n : n - 1);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; ++i) {
                if (matIsLight) MS.lsrc.push_back(MS.fv0.size());

                MS.fv0.push_back(vs[0]);
                MS.fv1.push_back(vs[i]);
                MS.fv2.push_back(vs[i + 1]);

                MS.ft0.push_back(ts[0]);
                MS.ft1.push_back(ts[i]);
                MS.ft2.push_back(ts[i + 1]);

                MS.fn0.push_back(ns[0]);
                MS.fn1.push_back(ns[i]);
                MS.fn2.push_back(ns[i + 1]);

                MS.fm.push_back(matIdx);
            }

            // Expand the AABB
            for (int i = 0; i < vs.size(); ++i) {
                MS.O_AB_min_x = fminf(MS.O_AB_min_x, MS.vx[vs[i]]);
                MS.O_AB_min_y = fminf(MS.O_AB_min_y, MS.vy[vs[i]]);
                MS.O_AB_min_z = fminf(MS.O_AB_min_z, MS.vz[vs[i]]);
                MS.O_AB_max_x = fmaxf(MS.O_AB_max_x, MS.vx[vs[i]]);
                MS.O_AB_max_y = fmaxf(MS.O_AB_max_y, MS.vy[vs[i]]);
                MS.O_AB_max_z = fmaxf(MS.O_AB_max_z, MS.vz[vs[i]]);

                MS.SO_AB_min_x.back() = fminf(MS.SO_AB_min_x.back(), MS.vx[vs[i]]);
                MS.SO_AB_min_y.back() = fminf(MS.SO_AB_min_y.back(), MS.vy[vs[i]]);
                MS.SO_AB_min_z.back() = fminf(MS.SO_AB_min_z.back(), MS.vz[vs[i]]);
                MS.SO_AB_max_x.back() = fmaxf(MS.SO_AB_max_x.back(), MS.vx[vs[i]]);
                MS.SO_AB_max_y.back() = fmaxf(MS.SO_AB_max_y.back(), MS.vy[vs[i]]);
                MS.SO_AB_max_z.back() = fmaxf(MS.SO_AB_max_z.back(), MS.vz[vs[i]]);
            }
            continue;
        }

        else if (type == "vt") {
            float tx, ty; ss >> tx >> ty;
            MS.tx.push_back(tx);
            MS.ty.push_back(ty);
            continue;
        }

        else if (type == "vn") {
            float nx, ny, nz; ss >> nx >> ny >> nz;

            // Rotate the normal
            float x = nx, z = nz;
            nx = x * cos(yaw) - z * sin(yaw);
            nz = x * sin(yaw) + z * cos(yaw);

            float nm = sqrtf(nx * nx + ny * ny + nz * nz);

            MS.nx.push_back(nx / nm);
            MS.ny.push_back(ny / nm);
            MS.nz.push_back(nz / nm);
            continue;
        }

        else if (type == "o" || type == "g") {
            MS.SOrF.push_back(MS.fv0.size());
            MS.SO_AB_min_x.push_back(INFINITY);
            MS.SO_AB_min_y.push_back(INFINITY);
            MS.SO_AB_min_z.push_back(INFINITY);
            MS.SO_AB_max_x.push_back(-INFINITY);
            MS.SO_AB_max_y.push_back(-INFINITY);
            MS.SO_AB_max_z.push_back(-INFINITY);
            continue;
        }

        else if (type == "usemtl" || type == "AzMtl") {
            std::string matName;
            ss >> matName;

            matIdx = matMap[matName];
            matIsLight = MatMgr.h_mtls[matIdx].Ems_i > 0;
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

    MS.SOrF.push_back(MS.fv0.size());
    MS.SOrF.erase(MS.SOrF.begin());

    // ---------------------------------------------------------

    float shift_x = 0, shift_y = 0, shift_z = 0;
    // In the middle of the y-axis
    if (placement == 1) {
        shift_y = (MS.O_AB_min_y + MS.O_AB_max_y) / 2;
    }
    // On the floor
    else if (placement == 2) {
        shift_y = MS.O_AB_min_y;
    }
    // On the floor and in the dead center
    else if (placement == 3) {
        shift_y = MS.O_AB_min_y;
        shift_x = (MS.O_AB_min_x + MS.O_AB_max_x) / 2;
        shift_z = (MS.O_AB_min_z + MS.O_AB_max_z) / 2;
    }

    #pragma omp parallel for
    for (size_t i = 0; i < MS.vx.size(); ++i) {
        MS.vx[i] -= shift_x;
        MS.vy[i] -= shift_y;
        MS.vz[i] -= shift_z;
    }

    // // Shift the AABBs

    MS.O_AB_min_x -= shift_x;
    MS.O_AB_min_y -= shift_y;
    MS.O_AB_min_z -= shift_z;
    MS.O_AB_max_x -= shift_x;
    MS.O_AB_max_y -= shift_y;
    MS.O_AB_max_z -= shift_z;

    // ---------------------------------------------------------

    MeshMgr.append(MS);
}