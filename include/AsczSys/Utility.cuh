#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <AzStruct.cuh>

class Utils {
public:
    // Turn obj into .azb file
    static AzObj createAzb(
        const char *objPath, short placement,
        float scl, float yaw, float tX, float tY, float tZ
    );
};

#endif