#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <AsczTxtr.cuh>
#include <AsczMtl.cuh>
#include <AsczMesh.cuh>

#include <fstream>
#include <sstream>
#include <string>
#include <omp.h>
#include <unordered_map>

class Utils {
public:
    static void appendObj(
        AsczMesh &meshMgr, AsczMtl &matMgr, AsczTxtr &txtrMgr,
        const char *objPath, short placement=0, float scale=1.0f, short fIdxBased=1
    );
};

#endif