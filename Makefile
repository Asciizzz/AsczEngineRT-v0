all: ClearScreen AsczEngine Run

ClearScreen:
	rm -f AsczEngine.exe \
	clear

AsczEngine:
	nvcc\
		-I include/MathLib \
		-I include/Geometry \
		-I include/Graphic3D \
		-I include/Manager \
		-I include/System \
		\
		-I libraries/SFML/include \
		-L libraries/SFML/lib \
		\
		-o AsczEngine \
		\
		src/MathLib/Vector.cu \
		src/MathLib/Matrix.cu \
		src/Graphic3D/Ray.cu \
		src/Graphic3D/Camera.cu \
		src/Graphic3D/SFMLTexture.cu \
		src/Graphic3D/RayTrace.cu \
		src/Manager/MatManager.cu \
		src/Manager/TxtrManager.cu \
		src/Manager/MeshManager.cu \
		src/System/FpsHandler.cu \
		src/System/CsLogHandler.cu \
		src/System/Utility.cu \
		\
		AsczEngine.cu \
		\
		-lsfml-system \
		-lsfml-window \
		-lsfml-graphics \
		-lsfml-audio \
		-lopenal32 \
		\
		-arch=sm_61 \
		-rdc=true \
		--expt-relaxed-constexpr \
		--extended-lambda \
		--disable-warnings

Run:
	./AsczEngine

clean:
	rm -f AsczEngine.exe

# Add <-mwindows> so when you run AsczEngine.exe
# it doesnt open a terminal
# (unless you need debugging and stuff ofc)