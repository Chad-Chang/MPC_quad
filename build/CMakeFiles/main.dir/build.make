# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/simulate.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/simulate.cpp.o: ../src/simulate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/simulate.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/simulate.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/simulate.cpp

CMakeFiles/main.dir/src/simulate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/simulate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/simulate.cpp > CMakeFiles/main.dir/src/simulate.cpp.i

CMakeFiles/main.dir/src/simulate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/simulate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/simulate.cpp -o CMakeFiles/main.dir/src/simulate.cpp.s

CMakeFiles/main.dir/src/glfw_adapter.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/glfw_adapter.cpp.o: ../src/glfw_adapter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/glfw_adapter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/glfw_adapter.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_adapter.cpp

CMakeFiles/main.dir/src/glfw_adapter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/glfw_adapter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_adapter.cpp > CMakeFiles/main.dir/src/glfw_adapter.cpp.i

CMakeFiles/main.dir/src/glfw_adapter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/glfw_adapter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_adapter.cpp -o CMakeFiles/main.dir/src/glfw_adapter.cpp.s

CMakeFiles/main.dir/src/glfw_dispatch.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/glfw_dispatch.cpp.o: ../src/glfw_dispatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/src/glfw_dispatch.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/glfw_dispatch.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_dispatch.cpp

CMakeFiles/main.dir/src/glfw_dispatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/glfw_dispatch.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_dispatch.cpp > CMakeFiles/main.dir/src/glfw_dispatch.cpp.i

CMakeFiles/main.dir/src/glfw_dispatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/glfw_dispatch.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/glfw_dispatch.cpp -o CMakeFiles/main.dir/src/glfw_dispatch.cpp.s

CMakeFiles/main.dir/src/lodepng.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/lodepng.cpp.o: ../src/lodepng.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/src/lodepng.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/lodepng.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/lodepng.cpp

CMakeFiles/main.dir/src/lodepng.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/lodepng.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/lodepng.cpp > CMakeFiles/main.dir/src/lodepng.cpp.i

CMakeFiles/main.dir/src/lodepng.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/lodepng.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/lodepng.cpp -o CMakeFiles/main.dir/src/lodepng.cpp.s

CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o: ../src/platform_ui_adapter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/platform_ui_adapter.cpp

CMakeFiles/main.dir/src/platform_ui_adapter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/platform_ui_adapter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/platform_ui_adapter.cpp > CMakeFiles/main.dir/src/platform_ui_adapter.cpp.i

CMakeFiles/main.dir/src/platform_ui_adapter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/platform_ui_adapter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/platform_ui_adapter.cpp -o CMakeFiles/main.dir/src/platform_ui_adapter.cpp.s

CMakeFiles/main.dir/src/trajectory.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/trajectory.cpp.o: ../src/trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/main.dir/src/trajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/trajectory.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/trajectory.cpp

CMakeFiles/main.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/trajectory.cpp > CMakeFiles/main.dir/src/trajectory.cpp.i

CMakeFiles/main.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/trajectory.cpp -o CMakeFiles/main.dir/src/trajectory.cpp.s

CMakeFiles/main.dir/src/kinematics.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/kinematics.cpp.o: ../src/kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/main.dir/src/kinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/kinematics.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/kinematics.cpp

CMakeFiles/main.dir/src/kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/kinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/kinematics.cpp > CMakeFiles/main.dir/src/kinematics.cpp.i

CMakeFiles/main.dir/src/kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/kinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/kinematics.cpp -o CMakeFiles/main.dir/src/kinematics.cpp.s

CMakeFiles/main.dir/src/filter.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/filter.cpp.o: ../src/filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/main.dir/src/filter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/filter.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/filter.cpp

CMakeFiles/main.dir/src/filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/filter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/filter.cpp > CMakeFiles/main.dir/src/filter.cpp.i

CMakeFiles/main.dir/src/filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/filter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/filter.cpp -o CMakeFiles/main.dir/src/filter.cpp.s

CMakeFiles/main.dir/src/controller.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/controller.cpp.o: ../src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/main.dir/src/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/controller.cpp.o -c /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/controller.cpp

CMakeFiles/main.dir/src/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/controller.cpp > CMakeFiles/main.dir/src/controller.cpp.i

CMakeFiles/main.dir/src/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/src/controller.cpp -o CMakeFiles/main.dir/src/controller.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o" \
"CMakeFiles/main.dir/src/simulate.cpp.o" \
"CMakeFiles/main.dir/src/glfw_adapter.cpp.o" \
"CMakeFiles/main.dir/src/glfw_dispatch.cpp.o" \
"CMakeFiles/main.dir/src/lodepng.cpp.o" \
"CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o" \
"CMakeFiles/main.dir/src/trajectory.cpp.o" \
"CMakeFiles/main.dir/src/kinematics.cpp.o" \
"CMakeFiles/main.dir/src/filter.cpp.o" \
"CMakeFiles/main.dir/src/controller.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/main.cpp.o
main: CMakeFiles/main.dir/src/simulate.cpp.o
main: CMakeFiles/main.dir/src/glfw_adapter.cpp.o
main: CMakeFiles/main.dir/src/glfw_dispatch.cpp.o
main: CMakeFiles/main.dir/src/lodepng.cpp.o
main: CMakeFiles/main.dir/src/platform_ui_adapter.cpp.o
main: CMakeFiles/main.dir/src/trajectory.cpp.o
main: CMakeFiles/main.dir/src/kinematics.cpp.o
main: CMakeFiles/main.dir/src/filter.cpp.o
main: CMakeFiles/main.dir/src/controller.cpp.o
main: CMakeFiles/main.dir/build.make
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build /home/chad/Documents/mujoco-3.1.6/myproject/raibert_walking/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

