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

    static void saveAzObj(const char *dir, const char *name, AzObj OBJ);
};

#endif