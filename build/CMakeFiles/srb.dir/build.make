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
CMAKE_SOURCE_DIR = /home/chad/Documents/c++/casadi_practice

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chad/Documents/c++/casadi_practice/build

# Include any dependencies generated for this target.
include CMakeFiles/srb.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/srb.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/srb.dir/flags.make

CMakeFiles/srb.dir/SRB.cpp.o: CMakeFiles/srb.dir/flags.make
CMakeFiles/srb.dir/SRB.cpp.o: ../SRB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/c++/casadi_practice/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/srb.dir/SRB.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srb.dir/SRB.cpp.o -c /home/chad/Documents/c++/casadi_practice/SRB.cpp

CMakeFiles/srb.dir/SRB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srb.dir/SRB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Documents/c++/casadi_practice/SRB.cpp > CMakeFiles/srb.dir/SRB.cpp.i

CMakeFiles/srb.dir/SRB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srb.dir/SRB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Documents/c++/casadi_practice/SRB.cpp -o CMakeFiles/srb.dir/SRB.cpp.s

# Object files for target srb
srb_OBJECTS = \
"CMakeFiles/srb.dir/SRB.cpp.o"

# External object files for target srb
srb_EXTERNAL_OBJECTS =

srb: CMakeFiles/srb.dir/SRB.cpp.o
srb: CMakeFiles/srb.dir/build.make
srb: CMakeFiles/srb.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chad/Documents/c++/casadi_practice/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable srb"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srb.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/srb.dir/build: srb

.PHONY : CMakeFiles/srb.dir/build

CMakeFiles/srb.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/srb.dir/cmake_clean.cmake
.PHONY : CMakeFiles/srb.dir/clean

CMakeFiles/srb.dir/depend:
	cd /home/chad/Documents/c++/casadi_practice/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chad/Documents/c++/casadi_practice /home/chad/Documents/c++/casadi_practice /home/chad/Documents/c++/casadi_practice/build /home/chad/Documents/c++/casadi_practice/build /home/chad/Documents/c++/casadi_practice/build/CMakeFiles/srb.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/srb.dir/depend

