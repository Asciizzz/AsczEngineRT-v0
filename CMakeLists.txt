cmake_minimum_required(VERSION 3.18)
project(AsczEngine LANGUAGES CUDA CXX)

# Enable CUDA as a language
enable_language(CUDA)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true --expt-relaxed-constexpr --extended-lambda -Xlinker -lgdiplus")

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set(CMAKE_VERBOSE_MAKEFILE OFF)


# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include/AsczMath
    ${CMAKE_SOURCE_DIR}/include/Ascz3D
    ${CMAKE_SOURCE_DIR}/include/AsczRT
    ${CMAKE_SOURCE_DIR}/include/AsczSys
)

set(CMAKE_CUDA_ARCHITECTURES 75)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR})
endforeach()


# Source files
add_executable(AsczEngine
    src/Ascz3D/AsczFrame.cu
    src/Ascz3D/AsczBvh.cu
    src/Ascz3D/AsczWin.cu
    src/Ascz3D/AsczCam.cu
    
    src/Ascz3D/AzStruct.cu

    src/AsczRT/RayCast.cu
    src/AsczRT/PathTraceSTD.cu
    src/AsczRT/PathTraceNEE.cu

    src/AsczSys/FpsHandler.cu
    src/AsczSys/Utility.cu

    AsczEngine.cu
)

set_property(TARGET AsczEngine PROPERTY CXX_STANDARD 17)