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
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/depend.make

# Include the progress variables for this target.
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/progress.make

# Include the compile flags for this target's objects.
include src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/flags.make

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/flags.make
src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o: ../src/osqp/lin_sys/direct/qdldl/qdldl_sources/examples/c/example.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eric/big-ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/qdldl_example.dir/examples/c/example.c.o   -c /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/examples/c/example.c

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/qdldl_example.dir/examples/c/example.c.i"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/examples/c/example.c > CMakeFiles/qdldl_example.dir/examples/c/example.c.i

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/qdldl_example.dir/examples/c/example.c.s"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources/examples/c/example.c -o CMakeFiles/qdldl_example.dir/examples/c/example.c.s

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.requires:

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.requires

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.provides: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.requires
	$(MAKE) -f src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/build.make src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.provides.build
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.provides

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.provides.build: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o


# Object files for target qdldl_example
qdldl_example_OBJECTS = \
"CMakeFiles/qdldl_example.dir/examples/c/example.c.o"

# External object files for target qdldl_example
qdldl_example_EXTERNAL_OBJECTS =

src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/qdldl_example: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o
src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/qdldl_example: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/build.make
src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/qdldl_example: src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a
src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/qdldl_example: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eric/big-ironhorse-ctrl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable out/qdldl_example"
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qdldl_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/build: src/osqp/lin_sys/direct/qdldl/qdldl_sources/out/qdldl_example

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/build

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/requires: src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/examples/c/example.c.o.requires

.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/requires

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/clean:
	cd /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -P CMakeFiles/qdldl_example.dir/cmake_clean.cmake
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/clean

src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/depend:
	cd /home/eric/big-ironhorse-ctrl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eric/big-ironhorse-ctrl /home/eric/big-ironhorse-ctrl/src/osqp/lin_sys/direct/qdldl/qdldl_sources /home/eric/big-ironhorse-ctrl/build /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources /home/eric/big-ironhorse-ctrl/build/src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/osqp/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldl_example.dir/depend

