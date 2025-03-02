cmake_minimum_required(VERSION 3.18)
project(AsczEngine LANGUAGES CUDA CXX)

# Enable CUDA as a language
enable_language(CUDA)

# Set CUDA Compilation Flags (similar to your Makefile)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true --expt-relaxed-constexpr --extended-lambda -Xlinker -lgdiplus")

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set(CMAKE_VERBOSE_MAKEFILE OFF)  # Suppress long compiler command output


# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include/AsczMath
    ${CMAKE_SOURCE_DIR}/include/Ascz3D
    ${CMAKE_SOURCE_DIR}/include/AsczRT
    ${CMAKE_SOURCE_DIR}/include/AsczSys
)

# Set CUDA architectures (modify based on GPU, 75 = Turing, 86 = Ampere)
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
    src/AsczMath/Vector.cu
    src/AsczMath/Matrix.cu

    src/AsczRT/RayTrace.cu

    src/Ascz3D/AsczMat.cu
    src/Ascz3D/AsczTxtr.cu
    src/Ascz3D/AsczMesh.cu
    src/Ascz3D/AsczBvh.cu
    src/Ascz3D/AsczWin.cu
    src/Ascz3D/AsczCam.cu

    src/AsczSys/FpsHandler.cu
    src/AsczSys/Utility.cu

    AsczEngine.cu
)

# Set C++ Standard
set_property(TARGET AsczEngine PROPERTY CXX_STANDARD 17)