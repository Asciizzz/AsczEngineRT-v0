#include <Utility.cuh>

#define VectF std::vector<float>

#define VectI std::vector<int>
#define VectLLI std::vector<LLInt>
#define VectULLI std::vector<ULLInt>

#define VectBool std::vector<bool>

#define VectStr std::vector<std::string>

void Utils::appendObj(
    MeshManager &meshMgr, BvhManager &bvhMgr,
    MatManager &matMgr, TxtrManager &txtrMgr,
    const char *objPath, short placement, float scale, short fIdxBased 
) {
    std::ifstream file(objPath);
    if (!file.is_open()) return;

    Vecs3f mv;
    Vecs2f mt;
    Vecs3f mn;

    Vecs3i mfv;
    Vecs3i mft;
    Vecs3i mfn;
    VectI mfm;
    VectI mfo;

    int matIdx = 0;
    std::unordered_map<std::string, int> matMap;

    int bvhIdx = -1;

    std::string path(objPath);

    // We will use these value to shift the mesh to desired position
    float minX = INFINITY, minY = INFINITY, minZ = INFINITY;
    float maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;

    VectStr lines;
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
            Vec3f v; ss >> v.x >> v.y >> v.z;
            v.scale(Vec3f(), scale);
            mv.push_back(v);

            minX = std::min(minX, v.x);
            minY = std::min(minY, v.y);
            minZ = std::min(minZ, v.z);
            maxX = std::max(maxX, v.x);
            maxY = std::max(maxY, v.y);
            maxZ = std::max(maxZ, v.z);
        }

        else if (type == "f") {
            Vec3i fv, ft, fn;

            VectI vs, ts, ns;
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

                // Note, setting it to fIdxBased - 1
                // Helps the operation below to work
                // Not exist = -1

                vs.push_back(v - fIdxBased);
                ts.push_back(t - fIdxBased);
                ns.push_back(n - fIdxBased);
            }

            // Triangulate the face
            for (int i = 1; i < vs.size() - 1; i++) {
                fv = Vec3i(vs[0], vs[i], vs[i + 1]);
                ft = Vec3i(ts[0], ts[i], ts[i + 1]);
                fn = Vec3i(ns[0], ns[i], ns[i + 1]);

                mfv.push_back(fv);
                mft.push_back(ft);
                mfn.push_back(fn);
                mfm.push_back(matIdx);
            }
        }

        else if (type == "vt") {
            Vec2f t; ss >> t.x >> t.y;
            mt.push_back(t);
        }

        else if (type == "vn") {
            Vec3f n; ss >> n.x >> n.y >> n.z;
            n.norm(); // Just in case
            mn.push_back(n);
        }

        else if (type == "o") {
            mfo.push_back(mfv.size());

            bvhIdx = bvhMgr.appendNode(BvhNode());
            bvhMgr.h_nodes[bvhIdx].fl = mfv.size();

            if (bvhIdx > 0) {
                bvhMgr.h_nodes[bvhIdx - 1].fr = mfv.size();
            }
        }

        else if (type == "usemtl") {
            std::string matName;
            ss >> matName;

            matIdx = matMap[matName];
        }

        else if (type == "mtllib") {
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

                if (mtlType == "newmtl") {
                    matIdx = matMgr.appendMaterial(Material());
                    std::string matName; mtlSS >> matName;
                    matMap[matName] = matIdx;
                }

                if (mtlType == "Kd") {
                    Vec3f Kd; mtlSS >> Kd.x >> Kd.y >> Kd.z;
                    matMgr.h_mats[matIdx].Kd = Kd;
                }

                if (mtlType == "map_Kd") {
                    std::string txtrPath; mtlSS >> txtrPath;

                    matMgr.h_mats[matIdx].mapKd = txtrMgr.appendTexture(
                        (mtlDir + txtrPath).c_str()
                    );
                }

                // Additional attributes can be added here
                // Even those that do no exist in a typical .mtl file
                // for debugging of course

                // Those that do not exist in a typical .mtl file
                if (mtlType == "refl") {
                    float refl; mtlSS >> refl;
                    matMgr.h_mats[matIdx].reflect = refl;
                } else if (mtlType == "transm") {
                    float transm; mtlSS >> transm;
                    matMgr.h_mats[matIdx].transmit = transm;
                } else if (mtlType == "Fresnel") {
                    float fresnel; mtlSS >> fresnel;
                    matMgr.h_mats[matIdx].Fresnel = fresnel;
                } else if (mtlType == "Phong") { // 0: no, 1: yes
                    int phong; mtlSS >> phong;
                    matMgr.h_mats[matIdx].Phong = phong;
                }
            }
        }
    }
    mfo.push_back(mfv.size());
    mfo.erase(mfo.begin());

    bvhMgr.h_nodes.back().fr = mfv.size();

    #pragma omp parallel for
    for (size_t i = 0; i < mv.size(); i++) {
        // Shift to center of xz plane
        if (placement > 0) {
            mv[i].x -= (minX + maxX) / 2;
            mv[i].z -= (minZ + maxZ) / 2;
        }

        // Shift to center
        if (placement == 1) mv[i].y -= minY;
        // Shift to floor (y = 0)
        else if (placement == 2) mv[i].y -= minY;
    }

    #pragma omp parallel for
    for (int n = 0; n < bvhMgr.num; n++) {
        BvhNode &node = bvhMgr.h_nodes[n];

        for (int i = node.fl; i < node.fr; i++) {
            Vec3i fv = mfv[i];

            node.recalc(mv[fv.x]);
            node.recalc(mv[fv.y]);
            node.recalc(mv[fv.z]);
        }
    }

    MeshStruct mesh;
    mesh.v = mv;
    mesh.t = mt;
    mesh.n = mn;
    mesh.fv = mfv;
    mesh.ft = mft;
    mesh.fn = mfn;
    mesh.fm = mfm;
    mesh.fo = mfo;

    meshMgr.appendMesh(mesh);
}