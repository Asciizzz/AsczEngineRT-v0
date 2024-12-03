all: ClearScreen AsczEngine Run

ClearScreen:
	rm -f AsczEngine.exe \
	clear

AsczEngine:
	nvcc \
		\
		-I libraries/SFML/include \
		-L libraries/SFML/lib \
		\
		-o AsczEngine \
		\
		\
		AsczEngine.cu \
		\
		-lsfml-system \
		-lsfml-window \
		-lsfml-graphics \
		-lsfml-audio \
		-lopenal32 \
		\
		-rdc=true \
		--expt-relaxed-constexpr \
		--extended-lambda \

Run:
	./AsczEngine

clean:
	rm -f AsczEngine.exe

# Add <-mwindows> so when you run AsczEngine.exe
# it doesnt open a terminal
# (unless you need debugging and stuff ofc)