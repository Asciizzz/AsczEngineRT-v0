# Set CUDA and OptiX paths
CUDA_PATH = "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6"
OPTIX_SDK_PATH = "C:/ProgramData/NVIDIA Corporation/OptiX SDK 8.1.0"

# Compiler and linker flags
NVCC = $(CUDA_PATH)/bin/nvcc
CXXFLAGS = -I$(OPTIX_SDK_PATH)/include -I$(CUDA_PATH)/include
LDFLAGS = -L$(CUDA_PATH)/lib/x64

# Sources and target
SRC = src/main.cu
OBJ = main.o
TARGET = raytracer

# Build rules
all: $(TARGET)

$(TARGET): $(OBJ)
	$(NVCC) $(OBJ) -o $(TARGET) $(LDFLAGS) -lcudart_static

%.o: %.cu
	$(NVCC) -c $< -o $@ $(CXXFLAGS)

clean:
	del $(OBJ) $(TARGET)
