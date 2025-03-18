#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <AzStruct.cuh>

class Utils {
public:
    // BETA
    static AzObj createAzObj(
        const char *objPath, short placement,
        float scl, float yaw, float tX, float tY, float tZ
    );
};

#endif