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
CMAKE_SOURCE_DIR = /home/eric/big-ironhorse-ctrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eric/big-ironhorse-ctrl/build

# Include any dependencies generated for this target.
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/depend.make

# Include the progress variables for this target.
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/progress.make

# Include the compile flags for this target's objects.
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/flags.make

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/flags.make
src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o: ../src/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/big-ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/qdldl.dir/src/qdldl.c.o   -c /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/qdldl.dir/src/qdldl.c.i"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c > CMakeFiles/qdldl.dir/src/qdldl.c.i

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/qdldl.dir/src/qdldl.c.s"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c -o CMakeFiles/qdldl.dir/src/qdldl.c.s

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.requires:

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.requires

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.provides: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.requires
	$(MAKE) -f src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/build.make src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.provides.build
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.provides

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.provides.build: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o


# Object files for target qdldl
qdldl_OBJECTS = \
"CMakeFiles/qdldl.dir/src/qdldl.c.o"

# External object files for target qdldl
qdldl_EXTERNAL_OBJECTS =

src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o
src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/build.make
src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eric/big-ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library out/libqdldl.so"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qdldl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/build: src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/build

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/requires: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/src/qdldl.c.o.requires

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/requires

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/clean:
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -P CMakeFiles/qdldl.dir/cmake_clean.cmake
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/clean

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/depend:
	cd /home/eric/big-ironhorse-ctrl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eric/big-ironhorse-ctrl /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources /home/eric/big-ironhorse-ctrl/build /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl.dir/depend

