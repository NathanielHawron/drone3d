# @TODO: Include windows compatibility & windows test on Linux through proton
# @TODO: microoptimize build system and features, and include CLI compatibility/build option diagnostic

CXX = g++ -DIMGUI_DEFINE_MATH_OPERATORS #-std=c++20
GLFW_MACRO = _GLFW_WAYLAND

OBJ_RAW = glad.o imgui_demo.o imnf.o file_dialogue.o
OBJ_RAW_ = drone.o ground.o
OBJ = $(addprefix ./.o/vendor/, $(OBJ_RAW)) $(addprefix ./.o/, $(OBJ_RAW_))

LIB_RAW = libglfw3.a imgui.a libvision_lib_ARCH.a
LIB =  $(addprefix ./.a/, $(LIB_RAW))

INCLUDE_RAW = /glad/include /glfw/include /glm / /imgui /ImNodeFlow/include /NRA_visionGL/include /NRA_visionGL/build/include /NRA_visionGL/vendor
INCLUDE = $(addprefix -I ./submodule,$(INCLUDE_RAW)) -I ./include

run: build
	./.bin/main

buildo: $(patsubst ./src/%.cpp, ./.o/%.o, $(wildcard ./src/*.cpp))
./.o/%.o: ./src/%.cpp
	-@echo -e "\033[0;32mBuilding $@\033[0;36m"
	$(CXX) $< -o $@ -c $(INCLUDE) $(LIB)
	@echo -e "\033[0;32mBuilt $@\033[0m"

build: buildo ./main.cpp ./include/*
	$(CXX) $(INCLUDE) ./main.cpp $(OBJ) $(LIB) -o ./.bin/main


init_dir:
	-mkdir ./.o
	-mkdir ./.o/vendor
	-mkdir ./.a
	-mkdir ./submodule/.o

build_deps: init_dir build_GLAD build_GLFW build_IMNF build_ImGui build_NRA_vision build_FileDialogue
	@echo "Dependencies built"

build_GLAD: ./.o/vendor/glad.o
./.o/vendor/glad.o:
	$(CXX) ./submodule/glad/src/glad.c -c -o ./.o/vendor/glad.o -I ./submodule/glad/include
build_GLFW: ./.a/libglfw3.a
./.a/libglfw3.a:
	cd submodule && cmake -S ./glfw -B ./glfw_build && cmake --build ./glfw_build
	cp ./submodule/glfw_build/src/libglfw3.a ./.a/libglfw3.a
build_IMNF: ./.o/vendor/imnf.o
./.o/vendor/imnf.o:
	$(CXX) ./submodule/ImNodeFlow/src/ImNodeFlow.cpp -c -o ./.o/vendor/imnf.o -I ./submodule/ImNodeFlow/include -I ./submodule/imgui
build_ImGui: ./.a/imgui
./.a/imgui:
	$(CXX) ./submodule/imgui/imgui_demo.cpp -o ./.o/vendor/imgui_demo.o -I ./submodule/imgui -c
	$(CXX) ./submodule/imgui/imgui_draw.cpp -o ./submodule/.o/imgui_draw.o -I ./submodule/imgui -c
	$(CXX) ./submodule/imgui/imgui_tables.cpp -o ./submodule/.o/imgui_tables.o -I ./submodule/imgui -c
	$(CXX) ./submodule/imgui/imgui_widgets.cpp -o ./submodule/.o/imgui_widgets.o -I ./submodule/imgui -c
	$(CXX) ./submodule/imgui/imgui.cpp -o ./submodule/.o/imgui.o -I ./submodule/imgui -c
# Backend Files
	$(CXX) ./submodule/imgui/backends/imgui_impl_glfw.cpp -o ./submodule/.o/imgui_impl_glfw.o -I ./submodule/imgui -c
	$(CXX) ./submodule/imgui/backends/imgui_impl_opengl3.cpp -o ./submodule/.o/imgui_impl_opengl3.o -I ./submodule/imgui -c

	ar -rcs ./.a/imgui.a ./submodule/.o/imgui*.o
build_NRA_vision: ./.a/libvision_lib_ARCH.a
./.a/libvision_lib_ARCH.a:
	cd ./submodule/NRA_visionGL && make init && make build
	cp ./submodule/NRA_visionGL/lib/libvision_lib_ARCH.a ./.a/libvision_lib_ARCH.a
build_FileDialogue: ./.o/vendor/file_dialogue.o
./.o/vendor/file_dialogue.o:
	$(CXX) ./submodule/FileExplorer/src/file_dialogue.cpp -o ./.o/vendor/file_dialogue.o -I ./submodule/FileExplorer/include -I ./submodule -c