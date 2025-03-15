#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <AsczTxtr.cuh>
#include <AsczMat.cuh>
#include <AsczMesh.cuh>

class Utils {
public:
    static void appendObj(
        AsczMesh &meshMgr, AsczMat &matMgr, AsczTxtr &txtrMgr,
        const char *objPath, short placement=0,
        float scale=1.0f, float yaw=0.0f,
        float tX=0.0f, float tY=0.0f, float tZ=0.0f
    );
};

#endif