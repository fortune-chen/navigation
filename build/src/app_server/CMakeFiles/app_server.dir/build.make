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
CMAKE_SOURCE_DIR = /home/chen/project/fl-slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chen/project/fl-slam/build

# Include any dependencies generated for this target.
include src/app_server/CMakeFiles/app_server.dir/depend.make

# Include the progress variables for this target.
include src/app_server/CMakeFiles/app_server.dir/progress.make

# Include the compile flags for this target's objects.
include src/app_server/CMakeFiles/app_server.dir/flags.make

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o: src/app_server/CMakeFiles/app_server.dir/flags.make
src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o: ../src/app_server/app_server_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_server.dir/app_server_main.cpp.o -c /home/chen/project/fl-slam/src/app_server/app_server_main.cpp

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_server.dir/app_server_main.cpp.i"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/project/fl-slam/src/app_server/app_server_main.cpp > CMakeFiles/app_server.dir/app_server_main.cpp.i

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_server.dir/app_server_main.cpp.s"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/project/fl-slam/src/app_server/app_server_main.cpp -o CMakeFiles/app_server.dir/app_server_main.cpp.s

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.requires:

.PHONY : src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.requires

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.provides: src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.requires
	$(MAKE) -f src/app_server/CMakeFiles/app_server.dir/build.make src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.provides.build
.PHONY : src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.provides

src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.provides.build: src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o


# Object files for target app_server
app_server_OBJECTS = \
"CMakeFiles/app_server.dir/app_server_main.cpp.o"

# External object files for target app_server
app_server_EXTERNAL_OBJECTS =

src/app_server/app_server: src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o
src/app_server/app_server: src/app_server/CMakeFiles/app_server.dir/build.make
src/app_server/app_server: src/app_server/libapp_server_lib.a
src/app_server/app_server: src/dds_wrapper/libdds_wrapper.so
src/app_server/app_server: src/app_server/CMakeFiles/app_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable app_server"
	cd /home/chen/project/fl-slam/build/src/app_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/app_server/CMakeFiles/app_server.dir/build: src/app_server/app_server

.PHONY : src/app_server/CMakeFiles/app_server.dir/build

src/app_server/CMakeFiles/app_server.dir/requires: src/app_server/CMakeFiles/app_server.dir/app_server_main.cpp.o.requires

.PHONY : src/app_server/CMakeFiles/app_server.dir/requires

src/app_server/CMakeFiles/app_server.dir/clean:
	cd /home/chen/project/fl-slam/build/src/app_server && $(CMAKE_COMMAND) -P CMakeFiles/app_server.dir/cmake_clean.cmake
.PHONY : src/app_server/CMakeFiles/app_server.dir/clean

src/app_server/CMakeFiles/app_server.dir/depend:
	cd /home/chen/project/fl-slam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chen/project/fl-slam /home/chen/project/fl-slam/src/app_server /home/chen/project/fl-slam/build /home/chen/project/fl-slam/build/src/app_server /home/chen/project/fl-slam/build/src/app_server/CMakeFiles/app_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/app_server/CMakeFiles/app_server.dir/depend

