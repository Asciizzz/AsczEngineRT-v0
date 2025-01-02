#include <Utility.cuh>

std::vector<Triangle> Utils::readObjFile(std::string name, std::string path, short fIdxBased, short placement) {
    std::ifstream file(path);
    if (!file.is_open()) return {};

    std::string line;

    VectF wx, wy, wz;
    VectF tu, tv;
    VectF nx, ny, nz;
    VectULLI fw; VectLLI ft, fn; // x3

    // We will use these value to shift the mesh to the origin
    float minX = INFINITY, minY = INFINITY, minZ = INFINITY;
    float maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;

    // Extract the lines from the file
    VectStr lines;
    while (std::getline(file, line)) {
        if (line.size() == 0 || line[0] == '#') continue;
        lines.push_back(line);
    }

    bool off = false; // Mostly for debugging purposes

    #pragma omp parallel for collapse(2)
    for (size_t i = 0; i < lines.size(); i++) {
        std::stringstream ss(lines[i]);
        std::string type;
        ss >> type;

        if (type == "off") off = true;
        if (type == "on") off = false;
        if (off) continue;

        if (type == "v") {
            Vec3f v;
            ss >> v.x >> v.y >> v.z;

            // Update the min and max values
            minX = std::min(minX, v.x);
            minY = std::min(minY, v.y);
            minZ = std::min(minZ, v.z);
            maxX = std::max(maxX, v.x);
            maxY = std::max(maxY, v.y);
            maxZ = std::max(maxZ, v.z);

            wx.push_back(v.x);
            wy.push_back(v.y);
            wz.push_back(v.z);
        } else if (type == "vt") {
            Vec2f t;
            ss >> t.x >> t.y;
            tu.push_back(t.x);
            tv.push_back(t.y);
        } else if (type == "vn") {
            Vec3f n;
            ss >> n.x >> n.y >> n.z;
            n.norm();

            nx.push_back(n.x);
            ny.push_back(n.y);
            nz.push_back(n.z);
        } else if (type == "f") {
            VectULLI vs;
            VectLLI ts, ns;
            while (ss.good()) {
                std::string vtn;
                ss >> vtn;

                ULLInt v;
                LLInt t = 0, n = 0;
                std::stringstream ss2(vtn);

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

                vs.push_back(v - fIdxBased); // Adjust to 0-based index
                ts.push_back(t - fIdxBased); // Adjust to 0-based index
                ns.push_back(n - fIdxBased); // Adjust to 0-based index
            }

            /* For n points, we will construct n - 2 triangles
            
            Example: (A B C D E F):
            - Use A as anchor point
                => (A B C), (A C D), (A D E), (A E F)

            Note:  .obj files are assumed to organized the points
                    in a clockwise (or counter-clockwise) order
                    If they don't, well, sucks to be you
            */

            for (int i = 1; i < vs.size() - 1; i++) {
                fw.push_back(vs[0]); fw.push_back(vs[i]); fw.push_back(vs[i + 1]);
                ft.push_back(ts[0]); ft.push_back(ts[i]); ft.push_back(ts[i + 1]);
                fn.push_back(ns[0]); fn.push_back(ns[i]); fn.push_back(ns[i + 1]);
            }
        }
    }

    #pragma omp parallel for
    for (size_t i = 0; i < wx.size(); i++) {
        // Shift to center of xz plane
        if (placement > 0) {
            wx[i] -= (minX + maxX) / 2;
            wz[i] -= (minZ + maxZ) / 2;
        }

        if (placement == 1) { // Shift to center
            wy[i] -= (minY + maxY) / 2;
        } else if (placement == 2) { // Shift to floor
            wy[i] -= minY;
        }
    }

    // For every face, we will create a triangle
    std::vector<Triangle> tris;
    for (size_t i = 0; i < fw.size(); i += 3) {
        Triangle tri;

        tri.v0 = Vec3f(wx[fw[i]], wy[fw[i]], wz[fw[i]]);
        tri.v1 = Vec3f(wx[fw[i + 1]], wy[fw[i + 1]], wz[fw[i + 1]]);
        tri.v2 = Vec3f(wx[fw[i + 2]], wy[fw[i + 2]], wz[fw[i + 2]]);

        tri.n0 = Vec3f(nx[fn[i]], ny[fn[i]], nz[fn[i]]);
        tri.n1 = Vec3f(nx[fn[i + 1]], ny[fn[i + 1]], nz[fn[i + 1]]);
        tri.n2 = Vec3f(nx[fn[i + 2]], ny[fn[i + 2]], nz[fn[i + 2]]);

        // tri.uniformColor(Vec3f(1, 1, 1));
        // Give each vertex a color based on the ratio of min and max values
        // With range from m to r + m
        float m = 0.6, r = 0.4;
        tri.c0 = Vec3f(tri.v0.x / maxX * r + m, tri.v0.y / maxY * r + m, tri.v0.z / maxZ * r + m);
        tri.c1 = Vec3f(tri.v1.x / maxY * r + m, tri.v1.y / maxZ * r + m, tri.v1.z / maxX * r + m);
        tri.c2 = Vec3f(tri.v2.x / maxZ * r + m, tri.v2.y / maxX * r + m, tri.v2.z / maxY * r + m);

        tris.push_back(tri);
    }

    return tris;
}