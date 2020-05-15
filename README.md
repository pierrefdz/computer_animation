# INF585 VCL

## Setup compilation in command line using the Makefile (in Linux only)

$ make

$ ./pgm

## Setup compilation in command line using CMake (in Linux/MacOs)

This step create the build directory, call CMake to generate the Makefile, compile and execute the code. Note that the creation of the build directory and CMake call has to be done only once on a given system.

The following command assume you have opened a command line in the directory vcl/

### Create a build directory

$ mkdir build

$ cd build

### Execute CMake, compile

$ cmake ..

$ make

$ cd ..

### Execute

$ build/pgm

(note: the build directory is temporary and can be removed safely when switching between different computers)

### Note on Compilation / Execution 

When editing the source code (without adding/removing files), you don't need to run CMake every time, but only call Makefile. The following command can be used from the vcl/ directory:

$ make -C build/

$ build/pgm


## Using QTCreator with CMake (Linux/MacOs)

Call qtcreator from vcl/ directory

$ qtcreator CMakeLists.txt &

Then follow the configuration steps from the GUI.

By default, a temporary directory build-cmake-Desktop-Default is created (in the parent directory of CMakeLists.txt file), as well as a file CMakeLists.txt.user (same directory than CMakeLists.txt file). Both can be removed safely.


## On Windows system with Visual Studio 

- Use CMakeLists.txt with Visual Studio
- Precompiled version of GLFW3 is provided (precompiled/glfw3_win)
- You need to copy data/ and shaders/ directories in the executable directory



# PROJECT UP

## Files

The files created during the project are in the folder scenes>sources>up. This folder contains:
- sources and header files for "up" (the file where the scene is encoded), "boid" that controls the birds' behavior, "curtain" that controls the curtains' behavior and "balloon" that controls the balloons' behavior
- assets that contain the 3D models as .obj files and textures as .png files

## User manual

The user can:
- change the time scale on which the house is evolving by using the slider
- add or remove balloons by clicking on the add and remove buttons (not to fast, otherwise it generates freeze/ crash of the program)
- decide to display or not the house, the balloons and the birds
