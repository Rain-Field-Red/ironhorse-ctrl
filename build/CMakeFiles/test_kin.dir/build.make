# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/eric/ironhorse-ctrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eric/ironhorse-ctrl/build

# Include any dependencies generated for this target.
include CMakeFiles/test_kin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_kin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_kin.dir/flags.make

CMakeFiles/test_kin.dir/test/csvtools.cc.o: CMakeFiles/test_kin.dir/flags.make
CMakeFiles/test_kin.dir/test/csvtools.cc.o: ../test/csvtools.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_kin.dir/test/csvtools.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_kin.dir/test/csvtools.cc.o -c /home/eric/ironhorse-ctrl/test/csvtools.cc

CMakeFiles/test_kin.dir/test/csvtools.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_kin.dir/test/csvtools.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eric/ironhorse-ctrl/test/csvtools.cc > CMakeFiles/test_kin.dir/test/csvtools.cc.i

CMakeFiles/test_kin.dir/test/csvtools.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_kin.dir/test/csvtools.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eric/ironhorse-ctrl/test/csvtools.cc -o CMakeFiles/test_kin.dir/test/csvtools.cc.s

CMakeFiles/test_kin.dir/test/csvtools.cc.o.requires:

.PHONY : CMakeFiles/test_kin.dir/test/csvtools.cc.o.requires

CMakeFiles/test_kin.dir/test/csvtools.cc.o.provides: CMakeFiles/test_kin.dir/test/csvtools.cc.o.requires
	$(MAKE) -f CMakeFiles/test_kin.dir/build.make CMakeFiles/test_kin.dir/test/csvtools.cc.o.provides.build
.PHONY : CMakeFiles/test_kin.dir/test/csvtools.cc.o.provides

CMakeFiles/test_kin.dir/test/csvtools.cc.o.provides.build: CMakeFiles/test_kin.dir/test/csvtools.cc.o


CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o: CMakeFiles/test_kin.dir/flags.make
CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o: ../test/test_ironhorse_kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o -c /home/eric/ironhorse-ctrl/test/test_ironhorse_kinematics.cpp

CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eric/ironhorse-ctrl/test/test_ironhorse_kinematics.cpp > CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.i

CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eric/ironhorse-ctrl/test/test_ironhorse_kinematics.cpp -o CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.s

CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.requires:

.PHONY : CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.requires

CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.provides: CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_kin.dir/build.make CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.provides.build
.PHONY : CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.provides

CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.provides.build: CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o


# Object files for target test_kin
test_kin_OBJECTS = \
"CMakeFiles/test_kin.dir/test/csvtools.cc.o" \
"CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o"

# External object files for target test_kin
test_kin_EXTERNAL_OBJECTS =

test_kin: CMakeFiles/test_kin.dir/test/csvtools.cc.o
test_kin: CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o
test_kin: CMakeFiles/test_kin.dir/build.make
test_kin: /usr/local/lib/librbdl.so
test_kin: /usr/local/lib/librbdl_urdfreader.so
test_kin: /usr/lib/x86_64-linux-gnu/liblua5.1.so
test_kin: /usr/lib/x86_64-linux-gnu/libm.so
test_kin: CMakeFiles/test_kin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eric/ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_kin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_kin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_kin.dir/build: test_kin

.PHONY : CMakeFiles/test_kin.dir/build

CMakeFiles/test_kin.dir/requires: CMakeFiles/test_kin.dir/test/csvtools.cc.o.requires
CMakeFiles/test_kin.dir/requires: CMakeFiles/test_kin.dir/test/test_ironhorse_kinematics.cpp.o.requires

.PHONY : CMakeFiles/test_kin.dir/requires

CMakeFiles/test_kin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_kin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_kin.dir/clean

CMakeFiles/test_kin.dir/depend:
	cd /home/eric/ironhorse-ctrl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eric/ironhorse-ctrl /home/eric/ironhorse-ctrl /home/eric/ironhorse-ctrl/build /home/eric/ironhorse-ctrl/build /home/eric/ironhorse-ctrl/build/CMakeFiles/test_kin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_kin.dir/depend

