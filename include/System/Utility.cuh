#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <TxtrManager.cuh>
#include <MatManager.cuh>
#include <MeshManager.cuh>

class Utils {
public:
    static void appendObj(
        MeshManager &meshMgr, MatManager &matMgr, TxtrManager &txtrMgr,
        const char *objPath, short fIdxBased=1, short placement=0
    );
};

#endif