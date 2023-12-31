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

# Utility rule file for copy_codegen_srcs.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/copy_codegen_srcs.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/copy_codegen_srcs.dir/progress.make

src/CMakeFiles/copy_codegen_srcs: codegen_src/src/auxil.c
src/CMakeFiles/copy_codegen_srcs: codegen_src/src/error.c
src/CMakeFiles/copy_codegen_srcs: codegen_src/src/osqp_api.c
src/CMakeFiles/copy_codegen_srcs: codegen_src/src/scaling.c
src/CMakeFiles/copy_codegen_srcs: codegen_src/src/util.c

codegen_src/src/auxil.c: src/auxil.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying auxil.c"
	cd /home/jang/neuromeka_ws/src/osqp/src && /usr/bin/cmake -E copy /home/jang/neuromeka_ws/src/osqp/src/auxil.c /home/jang/neuromeka_ws/src/osqp/codegen_src/src/auxil.c

codegen_src/src/error.c: src/error.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Copying error.c"
	cd /home/jang/neuromeka_ws/src/osqp/src && /usr/bin/cmake -E copy /home/jang/neuromeka_ws/src/osqp/src/error.c /home/jang/neuromeka_ws/src/osqp/codegen_src/src/error.c

codegen_src/src/osqp_api.c: src/osqp_api.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Copying osqp_api.c"
	cd /home/jang/neuromeka_ws/src/osqp/src && /usr/bin/cmake -E copy /home/jang/neuromeka_ws/src/osqp/src/osqp_api.c /home/jang/neuromeka_ws/src/osqp/codegen_src/src/osqp_api.c

codegen_src/src/scaling.c: src/scaling.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Copying scaling.c"
	cd /home/jang/neuromeka_ws/src/osqp/src && /usr/bin/cmake -E copy /home/jang/neuromeka_ws/src/osqp/src/scaling.c /home/jang/neuromeka_ws/src/osqp/codegen_src/src/scaling.c

codegen_src/src/util.c: src/util.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jang/neuromeka_ws/src/osqp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Copying util.c"
	cd /home/jang/neuromeka_ws/src/osqp/src && /usr/bin/cmake -E copy /home/jang/neuromeka_ws/src/osqp/src/util.c /home/jang/neuromeka_ws/src/osqp/codegen_src/src/util.c

copy_codegen_srcs: codegen_src/src/auxil.c
copy_codegen_srcs: codegen_src/src/error.c
copy_codegen_srcs: codegen_src/src/osqp_api.c
copy_codegen_srcs: codegen_src/src/scaling.c
copy_codegen_srcs: codegen_src/src/util.c
copy_codegen_srcs: src/CMakeFiles/copy_codegen_srcs
copy_codegen_srcs: src/CMakeFiles/copy_codegen_srcs.dir/build.make
.PHONY : copy_codegen_srcs

# Rule to build all files generated by this target.
src/CMakeFiles/copy_codegen_srcs.dir/build: copy_codegen_srcs
.PHONY : src/CMakeFiles/copy_codegen_srcs.dir/build

src/CMakeFiles/copy_codegen_srcs.dir/clean:
	cd /home/jang/neuromeka_ws/src/osqp/src && $(CMAKE_COMMAND) -P CMakeFiles/copy_codegen_srcs.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/copy_codegen_srcs.dir/clean

src/CMakeFiles/copy_codegen_srcs.dir/depend:
	cd /home/jang/neuromeka_ws/src/osqp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp/src /home/jang/neuromeka_ws/src/osqp /home/jang/neuromeka_ws/src/osqp/src /home/jang/neuromeka_ws/src/osqp/src/CMakeFiles/copy_codegen_srcs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/copy_codegen_srcs.dir/depend
