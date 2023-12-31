# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jang/neuromeka_ws/src/osqp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jang/neuromeka_ws/src/osqp

# Include any dependencies generated for this target.
include CMakeFiles/osqpstatic.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/osqpstatic.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/osqpstatic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/osqpstatic.dir/flags.make

CMakeFiles/osqpstatic.dir/src/osqp_api.c.o: CMakeFiles/osqpstatic.dir/flags.make
CMakeFiles/osqpstatic.dir/src/osqp_api.c.o: src/osqp_api.c
CMakeFiles/osqpstatic.dir/src/osqp_api.c.o: CMakeFiles/osqpstatic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/osqpstatic.dir/src/osqp_api.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/osqpstatic.dir/src/osqp_api.c.o -MF CMakeFiles/osqpstatic.dir/src/osqp_api.c.o.d -o CMakeFiles/osqpstatic.dir/src/osqp_api.c.o -c /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c

CMakeFiles/osqpstatic.dir/src/osqp_api.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osqpstatic.dir/src/osqp_api.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c > CMakeFiles/osqpstatic.dir/src/osqp_api.c.i

CMakeFiles/osqpstatic.dir/src/osqp_api.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osqpstatic.dir/src/osqp_api.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c -o CMakeFiles/osqpstatic.dir/src/osqp_api.c.s

# Object files for target osqpstatic
osqpstatic_OBJECTS = \
"CMakeFiles/osqpstatic.dir/src/osqp_api.c.o"

# External object files for target osqpstatic
osqpstatic_EXTERNAL_OBJECTS = \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/auxil.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/error.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/scaling.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/util.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/polish.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/derivative.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/interrupt_unix.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/timing_linux.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/src/codegen.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/csc_math.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/csc_utils.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/algebra_libs.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/vector.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/builtin/matrix.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/SuiteSparse_config.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_1.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_2.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_aat.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_control.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_defaults.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_info.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_order.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_post_tree.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_postorder.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_preprocess.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_valid.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/kkt.c.o" \
"/home/jang/neuromeka_ws/src/osqp/CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/qdldl_interface.c.o" \
"/home/jang/neuromeka_ws/src/osqp/_deps/qdldl-build/CMakeFiles/qdldlobject.dir/src/qdldl.c.o"

out/libosqpstatic.a: CMakeFiles/osqpstatic.dir/src/osqp_api.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/auxil.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/error.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/scaling.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/util.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/polish.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/derivative.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/interrupt_unix.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/timing_linux.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/src/codegen.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/csc_math.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/csc_utils.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/builtin/algebra_libs.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/builtin/vector.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/builtin/matrix.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/SuiteSparse_config.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_1.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_2.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_aat.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_control.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_defaults.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_info.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_order.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_post_tree.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_postorder.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_preprocess.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/amd/src/amd_valid.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/kkt.c.o
out/libosqpstatic.a: CMakeFiles/OSQPLIB.dir/algebra/_common/lin_sys/qdldl/qdldl_interface.c.o
out/libosqpstatic.a: _deps/qdldl-build/CMakeFiles/qdldlobject.dir/src/qdldl.c.o
out/libosqpstatic.a: CMakeFiles/osqpstatic.dir/build.make
out/libosqpstatic.a: CMakeFiles/osqpstatic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library out/libosqpstatic.a"
	$(CMAKE_COMMAND) -P CMakeFiles/osqpstatic.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/osqpstatic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/osqpstatic.dir/build: out/libosqpstatic.a
.PHONY : CMakeFiles/osqpstatic.dir/build

CMakeFiles/osqpstatic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/osqpstatic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/osqpstatic.dir/clean

CMakeFiles/osqpstatic.dir/depend:
	cd /home/jang/neuromeka_ws/src/osqp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp/CMakeFiles/osqpstatic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/osqpstatic.dir/depend
