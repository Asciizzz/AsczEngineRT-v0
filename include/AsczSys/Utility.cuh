#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>

#include <fstream>
#include <sstream>
#include <string>
#include <omp.h>
#include <unordered_map>

class Utils {
public:
    static void appendObj(
        AsczMesh &meshMgr, AsczMat &matMgr, AsczTxtr &txtrMgr,
        const char *objPath, short placement=0,
        float scale=1.0f, float yaw=0.0f, Flt3 trans=0.0f
    );
};

#endif