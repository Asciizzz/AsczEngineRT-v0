#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <TxtrManager.cuh>
#include <MatManager.cuh>
#include <MeshManager.cuh>
#include <BvhManager.cuh>

#include <fstream>
#include <sstream>
#include <string>
#include <omp.h>
#include <unordered_map>
#include <iostream>

class Utils {
public:
    static void appendObj(
        MeshManager &meshMgr, BvhManager &bvhMgr,
        MatManager &matMgr, TxtrManager &txtrMgr,
        const char *objPath, short placement=0, float scale=1.0f, short fIdxBased=1
    );
};

#endif