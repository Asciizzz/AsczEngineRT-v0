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

    // BETA
    static AzObj createAzObj(
        const char *objPath, short placement,
        float scl, float yaw, float tX, float tY, float tZ
    );
};

#endif