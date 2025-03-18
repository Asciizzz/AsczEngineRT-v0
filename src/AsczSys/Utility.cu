#include <Utility.cuh>

#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
#include <chrono>

std::string timeHelper( std::chrono::high_resolution_clock::time_point start,
                        std::chrono::high_resolution_clock::time_point end) {
    return std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    ) + "ms";
}

AzObj Utils::createAzb(
    const char *objPath, short placement, bool save,
    float scl, float yaw, float tX, float tY, float tZ
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return AzObj();

    // Seperate the directory, the file name and the extension
    std::string path(objPath);
    std::string dir = path.substr(0, path.find_last_of("/\\") + 1);
    std::string name = path.substr(path.find_last_of("/\\") + 1);
    name = name.substr(0, name.find_last_of("."));

    std::string ext = path.substr(path.find_last_of(".") + 1);

    std::cout << dir << " " << name << " " << ext << "\n";

    AzObj OBJ;

    std::cout << "Obj " << objPath << " ...\n";
    auto objStart = std::chrono::high_resolution_clock::now();

    int matIdx = -1;
    bool matLight = false;

    // To avoid duplicate textures and materials
    // as well as to keep track of their indices
    std::unordered_map<std::string, int> TX_map;
    std::unordered_map<std::string, int> MT_map;

    std::string line;

    while (std::getline(file, line)) {
        if (line.size() == 0 || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string type; ss >> type;

        if (type == "v") {
            float vx, vy, vz; ss >> vx >> vy >> vz;
            vx *= scl; vy *= scl; vz *= scl;

            float x = vx, z = vz;
            vx = x * cos(yaw) - z * sin(yaw);
            vz = x * sin(yaw) + z * cos(yaw);

            OBJ.MS.vx.push_back(vx + tX);
            OBJ.MS.vy.push_back(vy + tY);
            OBJ.MS.vz.push_back(vz + tZ);

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

                vs.push_back(v < 0 ? OBJ.MS.vx.size() + v : v - 1);
                ts.push_back(t < 0 ? OBJ.MS.tx.size() + t : t - 1);
                ns.push_back(n < 0 ? OBJ.MS.nx.size() + n : n - 1);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; ++i) {
                if (matLight) OBJ.MS.lsrc.push_back(OBJ.MS.fv0.size());

                OBJ.MS.fv0.push_back(vs[0]);
                OBJ.MS.fv1.push_back(vs[i]);
                OBJ.MS.fv2.push_back(vs[i + 1]);

                OBJ.MS.ft0.push_back(ts[0]);
                OBJ.MS.ft1.push_back(ts[i]);
                OBJ.MS.ft2.push_back(ts[i + 1]);

                OBJ.MS.fn0.push_back(ns[0]);
                OBJ.MS.fn1.push_back(ns[i]);
                OBJ.MS.fn2.push_back(ns[i + 1]);

                OBJ.MS.fm.push_back(matIdx);
            }

            // Calculate the AABB
            for (int i = 0; i < vs.size(); ++i) {
                int v = vs[i]; 

                OBJ.AB_x = fminf(OBJ.AB_x, OBJ.MS.vx[v]);
                OBJ.AB_y = fminf(OBJ.AB_y, OBJ.MS.vy[v]);
                OBJ.AB_z = fminf(OBJ.AB_z, OBJ.MS.vz[v]);

                OBJ.AB_X = fmaxf(OBJ.AB_X, OBJ.MS.vx[v]);
                OBJ.AB_Y = fmaxf(OBJ.AB_Y, OBJ.MS.vy[v]);
                OBJ.AB_Z = fmaxf(OBJ.AB_Z, OBJ.MS.vz[v]);
            }

            continue;
        }

        else if (type == "vt") {
            float tx, ty; ss >> tx >> ty;
            OBJ.MS.tx.push_back(tx);
            OBJ.MS.ty.push_back(ty);
            continue;
        }

        else if (type == "vn") {
            float nx, ny, nz; ss >> nx >> ny >> nz;

            // Rotate the normal
            float x = nx, z = nz;
            nx = x * cos(yaw) - z * sin(yaw);
            nz = x * sin(yaw) + z * cos(yaw);

            float r_nm = 1 / sqrtf(nx * nx + ny * ny + nz * nz);

            OBJ.MS.nx.push_back(nx * r_nm);
            OBJ.MS.ny.push_back(ny * r_nm);
            OBJ.MS.nz.push_back(nz * r_nm);
            continue;
        }

        else if (type == "usemtl" || type == "AzMtl") {
            std::string matName; ss >> matName;

            matIdx = MT_map[matName];
            matLight = OBJ.MT.Ems_i[matIdx] > 0;
            continue;
        }

        else if (type == "mtllib" || type == "AzmLib") {
            std::string mtlPath; ss >> mtlPath;

            auto mtlStart = std::chrono::high_resolution_clock::now();
            std::cout << "| Mtl " << mtlPath << " ...\n";

            std::ifstream mtlFile(dir + mtlPath);
            if (!mtlFile.is_open()) continue;

            std::string mtlLine;
            while (std::getline(mtlFile, mtlLine)) {
                if (mtlLine.size() == 0 || mtlLine[0] == '#') continue;

                std::stringstream MTss(mtlLine);
                std::string mtlType;
                MTss >> mtlType;

                if (mtlType == "newmtl" || mtlType == "AzMtl") {
                    std::string matName; MTss >> matName;

                    if (MT_map.find(matName) != MT_map.end()) {
                        matIdx = MT_map[matName];
                        continue;
                    }

                    matIdx = OBJ.MT.push();
                    MT_map[matName] = matIdx;
                }
                // Albedo
                else if (mtlType == "Kd" || mtlType == "Alb") {
                    MTss >> OBJ.MT.Alb_r[matIdx] >>
                            OBJ.MT.Alb_g[matIdx] >>
                            OBJ.MT.Alb_b[matIdx];
                }
                // Albedo map
                else if (mtlType == "map_Kd" || mtlType == "AlbMap") {
                    std::string TX_name; MTss >> TX_name;

                    // Search for existing texture
                    if (TX_map.find(TX_name) != TX_map.end()) {
                        OBJ.MT.AlbMap[matIdx] = TX_map[TX_name];
                        continue;
                    }

                    auto txtrStart = std::chrono::high_resolution_clock::now();
                    std::cout << "| | Txtr " << TX_name << " ... ";

                    OBJ.MT.AlbMap[matIdx] = OBJ.TX.append((dir + TX_name).c_str());
                    auto txtrEnd = std::chrono::high_resolution_clock::now();

                    // Add to map
                    if (OBJ.MT.AlbMap[matIdx] != -1) {
                        TX_map[TX_name] = OBJ.MT.AlbMap[matIdx];
                        std::cout << "loaded in " << timeHelper(txtrStart, txtrEnd) << "\n";
                    } else std::cout << "failed\n";

                    continue;
                }
                // Roughness
                else if (mtlType == "Rough") {
                    MTss >> OBJ.MT.Rough[matIdx];
                }
                // Transmission
                else if (mtlType == "Tr") {
                    MTss >> OBJ.MT.Tr[matIdx];
                }
                else if (mtlType == "d") { // The opposite of Tr
                    MTss >> OBJ.MT.Tr[matIdx];
                    OBJ.MT.Tr[matIdx] = 1.0f - OBJ.MT.Tr[matIdx];
                }
                // Index of refraction
                else if (mtlType == "Ni" || mtlType == "Ior") {
                    MTss >> OBJ.MT.Ior[matIdx];
                }
                // Emission
                else if (mtlType == "Ke" || mtlType == "Ems") {
                    MTss >> OBJ.MT.Ems_r[matIdx] >>
                            OBJ.MT.Ems_g[matIdx] >>
                            OBJ.MT.Ems_b[matIdx] >>
                            OBJ.MT.Ems_i[matIdx];
                }
            }

            auto mtlEnd = std::chrono::high_resolution_clock::now();
            std::cout << "| Loaded in " << timeHelper(mtlStart, mtlEnd) << "\n";

            continue;
        }
    }

    OBJ.MS.v_num = OBJ.MS.vx.size();
    OBJ.MS.n_num = OBJ.MS.nx.size();
    OBJ.MS.t_num = OBJ.MS.tx.size();
    OBJ.MS.f_num = OBJ.MS.fv0.size();
    OBJ.MS.l_num = OBJ.MS.lsrc.size();

    float shift_x = 0, shift_y = 0, shift_z = 0;

    if (placement == 1) {
        shift_y = (OBJ.AB_y + OBJ.AB_Y) * 0.5f;
    }
    else if (placement == 2) {
        shift_y = OBJ.AB_y;
    }
    else if (placement == 3) {
        shift_y = OBJ.AB_y;
        shift_x = (OBJ.AB_x + OBJ.AB_X) * 0.5f;
        shift_z = (OBJ.AB_z + OBJ.AB_Z) * 0.5f;
    }

    #pragma omp parallel for
    for (size_t i = 0; i < OBJ.MS.vx.size(); ++i) {
        OBJ.MS.vx[i] -= shift_x;
        OBJ.MS.vy[i] -= shift_y;
        OBJ.MS.vz[i] -= shift_z;
    }

    OBJ.AB_x -= shift_x; OBJ.AB_y -= shift_y; OBJ.AB_z -= shift_z;
    OBJ.AB_X -= shift_x; OBJ.AB_Y -= shift_y; OBJ.AB_Z -= shift_z;


    // Debug:
    std::cout << "Result:\n";
    std::cout << "| Vertex: " << OBJ.MS.v_num << "\n";
    std::cout << "| Normal: " << OBJ.MS.n_num << "\n";
    std::cout << "| Texture: " << OBJ.MS.t_num << "\n";
    std::cout << "| Face: " << OBJ.MS.f_num << "\n";
    std::cout << "| Light: " << OBJ.MS.l_num << "\n";
    std::cout << "| Material: " << OBJ.MT.num << "\n";
    std::cout << "| Texture | Num: " << OBJ.TX.num <<
                " | Size: " << OBJ.TX.size << "\n";

    auto objEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Loaded in " << timeHelper(objStart, objEnd) << "\n";

    if (save) AzObj::save(OBJ, (dir + name + ".azb").c_str());

    return OBJ;
}