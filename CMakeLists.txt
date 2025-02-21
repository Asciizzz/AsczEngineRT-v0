cmake_minimum_required(VERSION 3.18)
project(AsczEngine LANGUAGES CUDA CXX)

# Enable CUDA as a language
enable_language(CUDA)

# Set CUDA Compilation Flags (similar to your Makefile)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true --expt-relaxed-constexpr --extended-lambda -lgdiplus -municode -mwindows -fexec-charset=UTF-8")

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_RESOLVE_DEVICE_SYMBOLS ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include/AsczMath
    ${CMAKE_SOURCE_DIR}/include/Ascz3D
    ${CMAKE_SOURCE_DIR}/include/Ascz3Dx
    ${CMAKE_SOURCE_DIR}/include/AsczSys
)

# Set CUDA architectures (modify based on GPU, 75 = Turing, 86 = Ampere)
set(CMAKE_CUDA_ARCHITECTURES 75)

# Source files
add_executable(AsczEngine
    src/AsczMath/Vector.cu
    src/AsczMath/Matrix.cu

    src/Ascz3D/Ray.cu
    src/Ascz3D/Camera.cu
    src/Ascz3D/RayTrace.cu

    src/Ascz3Dx/AsczMtl.cu
    src/Ascz3Dx/AsczTxtr.cu
    src/Ascz3Dx/AsczMesh.cu
    src/Ascz3Dx/AsczBvh.cu

    src/AsczSys/FpsHandler.cu
    src/AsczSys/Utility.cu

    AsczEngine.cu
)

# Copy .model and .cfg files
add_custom_command(TARGET AsczEngine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_SOURCE_DIR}/.model $<TARGET_FILE_DIR:AsczEngine>/.model
)

add_custom_command(TARGET AsczEngine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_SOURCE_DIR}/.cfg $<TARGET_FILE_DIR:AsczEngine>/.cfg
)

# Copy assets folder recursively
add_custom_command(TARGET AsczEngine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:AsczEngine>/assets
)


# Set C++ Standard
set_property(TARGET AsczEngine PROPERTY CXX_STANDARD 17)