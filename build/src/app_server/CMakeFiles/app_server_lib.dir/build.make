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
include src/app_server/CMakeFiles/app_server_lib.dir/depend.make

# Include the progress variables for this target.
include src/app_server/CMakeFiles/app_server_lib.dir/progress.make

# Include the compile flags for this target's objects.
include src/app_server/CMakeFiles/app_server_lib.dir/flags.make

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o: src/app_server/CMakeFiles/app_server_lib.dir/flags.make
src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o: ../src/app_server/flslam_api.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_server_lib.dir/flslam_api.cpp.o -c /home/chen/project/fl-slam/src/app_server/flslam_api.cpp

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_server_lib.dir/flslam_api.cpp.i"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/project/fl-slam/src/app_server/flslam_api.cpp > CMakeFiles/app_server_lib.dir/flslam_api.cpp.i

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_server_lib.dir/flslam_api.cpp.s"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/project/fl-slam/src/app_server/flslam_api.cpp -o CMakeFiles/app_server_lib.dir/flslam_api.cpp.s

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.requires:

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.requires

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.provides: src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.requires
	$(MAKE) -f src/app_server/CMakeFiles/app_server_lib.dir/build.make src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.provides.build
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.provides

src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.provides.build: src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o


src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o: src/app_server/CMakeFiles/app_server_lib.dir/flags.make
src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o: ../src/app_server/mapping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_server_lib.dir/mapping.cpp.o -c /home/chen/project/fl-slam/src/app_server/mapping.cpp

src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_server_lib.dir/mapping.cpp.i"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/project/fl-slam/src/app_server/mapping.cpp > CMakeFiles/app_server_lib.dir/mapping.cpp.i

src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_server_lib.dir/mapping.cpp.s"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/project/fl-slam/src/app_server/mapping.cpp -o CMakeFiles/app_server_lib.dir/mapping.cpp.s

src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.requires:

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.requires

src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.provides: src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.requires
	$(MAKE) -f src/app_server/CMakeFiles/app_server_lib.dir/build.make src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.provides.build
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.provides

src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.provides.build: src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o


src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o: src/app_server/CMakeFiles/app_server_lib.dir/flags.make
src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o: ../src/app_server/navigation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_server_lib.dir/navigation.cpp.o -c /home/chen/project/fl-slam/src/app_server/navigation.cpp

src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_server_lib.dir/navigation.cpp.i"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/project/fl-slam/src/app_server/navigation.cpp > CMakeFiles/app_server_lib.dir/navigation.cpp.i

src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_server_lib.dir/navigation.cpp.s"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/project/fl-slam/src/app_server/navigation.cpp -o CMakeFiles/app_server_lib.dir/navigation.cpp.s

src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.requires:

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.requires

src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.provides: src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.requires
	$(MAKE) -f src/app_server/CMakeFiles/app_server_lib.dir/build.make src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.provides.build
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.provides

src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.provides.build: src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o


src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o: src/app_server/CMakeFiles/app_server_lib.dir/flags.make
src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o: ../src/app_server/trimmer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/app_server_lib.dir/trimmer.cpp.o -c /home/chen/project/fl-slam/src/app_server/trimmer.cpp

src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app_server_lib.dir/trimmer.cpp.i"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chen/project/fl-slam/src/app_server/trimmer.cpp > CMakeFiles/app_server_lib.dir/trimmer.cpp.i

src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app_server_lib.dir/trimmer.cpp.s"
	cd /home/chen/project/fl-slam/build/src/app_server && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chen/project/fl-slam/src/app_server/trimmer.cpp -o CMakeFiles/app_server_lib.dir/trimmer.cpp.s

src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.requires:

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.requires

src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.provides: src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.requires
	$(MAKE) -f src/app_server/CMakeFiles/app_server_lib.dir/build.make src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.provides.build
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.provides

src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.provides.build: src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o


# Object files for target app_server_lib
app_server_lib_OBJECTS = \
"CMakeFiles/app_server_lib.dir/flslam_api.cpp.o" \
"CMakeFiles/app_server_lib.dir/mapping.cpp.o" \
"CMakeFiles/app_server_lib.dir/navigation.cpp.o" \
"CMakeFiles/app_server_lib.dir/trimmer.cpp.o"

# External object files for target app_server_lib
app_server_lib_EXTERNAL_OBJECTS =

src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o
src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o
src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o
src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o
src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/build.make
src/app_server/libapp_server_lib.a: src/app_server/CMakeFiles/app_server_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chen/project/fl-slam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libapp_server_lib.a"
	cd /home/chen/project/fl-slam/build/src/app_server && $(CMAKE_COMMAND) -P CMakeFiles/app_server_lib.dir/cmake_clean_target.cmake
	cd /home/chen/project/fl-slam/build/src/app_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app_server_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/app_server/CMakeFiles/app_server_lib.dir/build: src/app_server/libapp_server_lib.a

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/build

src/app_server/CMakeFiles/app_server_lib.dir/requires: src/app_server/CMakeFiles/app_server_lib.dir/flslam_api.cpp.o.requires
src/app_server/CMakeFiles/app_server_lib.dir/requires: src/app_server/CMakeFiles/app_server_lib.dir/mapping.cpp.o.requires
src/app_server/CMakeFiles/app_server_lib.dir/requires: src/app_server/CMakeFiles/app_server_lib.dir/navigation.cpp.o.requires
src/app_server/CMakeFiles/app_server_lib.dir/requires: src/app_server/CMakeFiles/app_server_lib.dir/trimmer.cpp.o.requires

.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/requires

src/app_server/CMakeFiles/app_server_lib.dir/clean:
	cd /home/chen/project/fl-slam/build/src/app_server && $(CMAKE_COMMAND) -P CMakeFiles/app_server_lib.dir/cmake_clean.cmake
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/clean

src/app_server/CMakeFiles/app_server_lib.dir/depend:
	cd /home/chen/project/fl-slam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chen/project/fl-slam /home/chen/project/fl-slam/src/app_server /home/chen/project/fl-slam/build /home/chen/project/fl-slam/build/src/app_server /home/chen/project/fl-slam/build/src/app_server/CMakeFiles/app_server_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/app_server/CMakeFiles/app_server_lib.dir/depend
