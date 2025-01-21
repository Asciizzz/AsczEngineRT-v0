#ifndef UTILITY_CUH
#define UTILITY_CUH

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Geom.cuh>

class Utils {
public:
    static std::vector<Geom> readObjFile(std::string name, std::string path, short fIdxBased=1, short placement=0);
};

#endif