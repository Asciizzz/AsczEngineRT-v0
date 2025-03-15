#include <Utility.cuh>

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

void Utils::appendObj(
    AsczMesh &MS, AsczMat &MT, AsczTxtr &TX,
    const char *objPath, short placement,
    float scale, float yaw, float tX, float tY, float tZ
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return;

    MeshStruct ms;

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

            ms.vx.push_back(vx + tX);
            ms.vy.push_back(vy + tY);
            ms.vz.push_back(vz + tZ);

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

                vs.push_back(v < 0 ? ms.vx.size() + v : v - 1);
                ts.push_back(t < 0 ? ms.tx.size() + t : t - 1);
                ns.push_back(n < 0 ? ms.nx.size() + n : n - 1);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; ++i) {
                if (matIsLight) ms.lsrc.push_back(ms.fv0.size());

                ms.fv0.push_back(vs[0]);
                ms.fv1.push_back(vs[i]);
                ms.fv2.push_back(vs[i + 1]);

                ms.ft0.push_back(ts[0]);
                ms.ft1.push_back(ts[i]);
                ms.ft2.push_back(ts[i + 1]);

                ms.fn0.push_back(ns[0]);
                ms.fn1.push_back(ns[i]);
                ms.fn2.push_back(ns[i + 1]);

                ms.fm.push_back(matIdx);
            }

            // Expand the AABB
            for (int i = 0; i < vs.size(); ++i) {
                ms.O_AB_min_x = fminf(ms.O_AB_min_x, ms.vx[vs[i]]);
                ms.O_AB_min_y = fminf(ms.O_AB_min_y, ms.vy[vs[i]]);
                ms.O_AB_min_z = fminf(ms.O_AB_min_z, ms.vz[vs[i]]);
                ms.O_AB_max_x = fmaxf(ms.O_AB_max_x, ms.vx[vs[i]]);
                ms.O_AB_max_y = fmaxf(ms.O_AB_max_y, ms.vy[vs[i]]);
                ms.O_AB_max_z = fmaxf(ms.O_AB_max_z, ms.vz[vs[i]]);

                // ms.SO_AB_min_x.back() = fminf(ms.SO_AB_min_x.back(), ms.vx[vs[i]]);
                // ms.SO_AB_min_y.back() = fminf(ms.SO_AB_min_y.back(), ms.vy[vs[i]]);
                // ms.SO_AB_min_z.back() = fminf(ms.SO_AB_min_z.back(), ms.vz[vs[i]]);
                // ms.SO_AB_max_x.back() = fmaxf(ms.SO_AB_max_x.back(), ms.vx[vs[i]]);
                // ms.SO_AB_max_y.back() = fmaxf(ms.SO_AB_max_y.back(), ms.vy[vs[i]]);
                // ms.SO_AB_max_z.back() = fmaxf(ms.SO_AB_max_z.back(), ms.vz[vs[i]]);
            }
            continue;
        }

        else if (type == "vt") {
            float tx, ty; ss >> tx >> ty;
            ms.tx.push_back(tx);
            ms.ty.push_back(ty);
            continue;
        }

        else if (type == "vn") {
            float nx, ny, nz; ss >> nx >> ny >> nz;

            // Rotate the normal
            float x = nx, z = nz;
            nx = x * cos(yaw) - z * sin(yaw);
            nz = x * sin(yaw) + z * cos(yaw);

            float nm = sqrtf(nx * nx + ny * ny + nz * nz);

            ms.nx.push_back(nx / nm);
            ms.ny.push_back(ny / nm);
            ms.nz.push_back(nz / nm);
            continue;
        }

        // else if (type == "o" || type == "g") {
        //     ms.SOrF.push_back(ms.fv0.size());
        //     ms.SO_AB_min_x.push_back(INFINITY);
        //     ms.SO_AB_min_y.push_back(INFINITY);
        //     ms.SO_AB_min_z.push_back(INFINITY);
        //     ms.SO_AB_max_x.push_back(-INFINITY);
        //     ms.SO_AB_max_y.push_back(-INFINITY);
        //     ms.SO_AB_max_z.push_back(-INFINITY);
        //     continue;
        // }

        else if (type == "usemtl" || type == "AzMtl") {
            std::string matName;
            ss >> matName;

            matIdx = matMap[matName];
            matIsLight = MT.h_mtls[matIdx].Ems_i > 0;
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
                    matIdx = MT.append(AzMtl(),
                        std::wstring(matName.begin(), matName.end()), 
                        std::wstring(mtlDir.begin(), mtlDir.end())
                    );
                    matMap[matName] = matIdx;
                }
                // Albedo
                else if (mtlType == "Kd" || mtlType == "Alb") {
                    float alb_r, alb_g, alb_b;
                    mtlSS >> alb_r >> alb_g >> alb_b;

                    MT.h_mtls[matIdx].Alb_r = alb_r;
                    MT.h_mtls[matIdx].Alb_g = alb_g;
                    MT.h_mtls[matIdx].Alb_b = alb_b;
                }
                // Albedo map
                else if (mtlType == "map_Kd" || mtlType == "AlbMap") {
                    std::string txtrPath; mtlSS >> txtrPath;

                    MT.h_mtls[matIdx].AlbMap = TX.appendTexture(
                        (mtlDir + txtrPath).c_str()
                    );
                }
                // Roughness
                else if (mtlType == "Rough") {
                    float Rough; mtlSS >> Rough;
                    MT.h_mtls[matIdx].Rough = Rough;
                }
                else if (mtlType == "Ns") { // Outdated
                    float Ns; mtlSS >> Ns;
                    MT.h_mtls[matIdx].Rough = 1.0f - Ns / 1000.0f;
                }
                // Transmission
                else if (mtlType == "Tr") {
                    float Tr; mtlSS >> Tr;
                    MT.h_mtls[matIdx].Tr = Tr;
                }
                else if (mtlType == "d") { // The opposite of Tr
                    float Tr; mtlSS >> Tr;
                    MT.h_mtls[matIdx].Tr = 1 - Tr;
                }
                // Index of refraction
                else if (mtlType == "Ni" || mtlType == "Ior") {
                    float Ior; mtlSS >> Ior;
                    MT.h_mtls[matIdx].Ior = Ior;
                }
                // Emission
                else if (mtlType == "Ke" || mtlType == "Ems") {
                    float Ems_r, Ems_g, Ems_b, Ems_i;
                    mtlSS >> Ems_r >> Ems_g >> Ems_b >> Ems_i;
                    Ems_i = Ems_i + !Ems_i; // In case the intensity is 0
                    MT.h_mtls[matIdx].Ems_r = Ems_r;
                    MT.h_mtls[matIdx].Ems_g = Ems_g;
                    MT.h_mtls[matIdx].Ems_b = Ems_b;
                    MT.h_mtls[matIdx].Ems_i = Ems_i;
                }

                // DEBUG VALUES
                else if (mtlType == "NoShade") {
                    MT.h_mtls[matIdx].NoShade = true;
                }
            }
            continue;
        }
    }

    // ms.SOrF.push_back(ms.fv0.size());
    // ms.SOrF.erase(ms.SOrF.begin());

    // ---------------------------------------------------------

    float shift_x = 0, shift_y = 0, shift_z = 0;
    // In the middle of the y-axis
    if (placement == 1) {
        shift_y = (ms.O_AB_min_y + ms.O_AB_max_y) / 2;
    }
    // On the floor
    else if (placement == 2) {
        shift_y = ms.O_AB_min_y;
    }
    // On the floor and in the dead center
    else if (placement == 3) {
        shift_y = ms.O_AB_min_y;
        shift_x = (ms.O_AB_min_x + ms.O_AB_max_x) / 2;
        shift_z = (ms.O_AB_min_z + ms.O_AB_max_z) / 2;
    }

    #pragma omp parallel for
    for (size_t i = 0; i < ms.vx.size(); ++i) {
        ms.vx[i] -= shift_x;
        ms.vy[i] -= shift_y;
        ms.vz[i] -= shift_z;
    }

    // Shift the AABBs

    ms.O_AB_min_x -= shift_x;
    ms.O_AB_min_y -= shift_y;
    ms.O_AB_min_z -= shift_z;
    ms.O_AB_max_x -= shift_x;
    ms.O_AB_max_y -= shift_y;
    ms.O_AB_max_z -= shift_z;

    // ---------------------------------------------------------

    MS.append(ms);
}