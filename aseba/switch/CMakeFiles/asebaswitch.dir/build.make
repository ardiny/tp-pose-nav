# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/admin1/install/aseba/aseba

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/admin1/install/aseba/build-aseba

# Include any dependencies generated for this target.
include switch/CMakeFiles/asebaswitch.dir/depend.make

# Include the progress variables for this target.
include switch/CMakeFiles/asebaswitch.dir/progress.make

# Include the compile flags for this target's objects.
include switch/CMakeFiles/asebaswitch.dir/flags.make

switch/CMakeFiles/asebaswitch.dir/switch.cpp.o: switch/CMakeFiles/asebaswitch.dir/flags.make
switch/CMakeFiles/asebaswitch.dir/switch.cpp.o: /home/admin1/install/aseba/aseba/switch/switch.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object switch/CMakeFiles/asebaswitch.dir/switch.cpp.o"
	cd /home/admin1/install/aseba/build-aseba/switch && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebaswitch.dir/switch.cpp.o -c /home/admin1/install/aseba/aseba/switch/switch.cpp

switch/CMakeFiles/asebaswitch.dir/switch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebaswitch.dir/switch.cpp.i"
	cd /home/admin1/install/aseba/build-aseba/switch && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/switch/switch.cpp > CMakeFiles/asebaswitch.dir/switch.cpp.i

switch/CMakeFiles/asebaswitch.dir/switch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebaswitch.dir/switch.cpp.s"
	cd /home/admin1/install/aseba/build-aseba/switch && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/switch/switch.cpp -o CMakeFiles/asebaswitch.dir/switch.cpp.s

switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.requires:
.PHONY : switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.requires

switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.provides: switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.requires
	$(MAKE) -f switch/CMakeFiles/asebaswitch.dir/build.make switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.provides.build
.PHONY : switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.provides

switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.provides.build: switch/CMakeFiles/asebaswitch.dir/switch.cpp.o

# Object files for target asebaswitch
asebaswitch_OBJECTS = \
"CMakeFiles/asebaswitch.dir/switch.cpp.o"

# External object files for target asebaswitch
asebaswitch_EXTERNAL_OBJECTS =

switch/asebaswitch: switch/CMakeFiles/asebaswitch.dir/switch.cpp.o
switch/asebaswitch: switch/CMakeFiles/asebaswitch.dir/build.make
switch/asebaswitch: libasebacore.a
switch/asebaswitch: /home/admin1/install/aseba/build-dashel/libdashel.a
switch/asebaswitch: /usr/lib/i386-linux-gnu/libudev.so
switch/asebaswitch: /home/admin1/install/aseba/build-dashel/libdashel.a
switch/asebaswitch: /usr/lib/i386-linux-gnu/libudev.so
switch/asebaswitch: switch/CMakeFiles/asebaswitch.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable asebaswitch"
	cd /home/admin1/install/aseba/build-aseba/switch && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/asebaswitch.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
switch/CMakeFiles/asebaswitch.dir/build: switch/asebaswitch
.PHONY : switch/CMakeFiles/asebaswitch.dir/build

switch/CMakeFiles/asebaswitch.dir/requires: switch/CMakeFiles/asebaswitch.dir/switch.cpp.o.requires
.PHONY : switch/CMakeFiles/asebaswitch.dir/requires

switch/CMakeFiles/asebaswitch.dir/clean:
	cd /home/admin1/install/aseba/build-aseba/switch && $(CMAKE_COMMAND) -P CMakeFiles/asebaswitch.dir/cmake_clean.cmake
.PHONY : switch/CMakeFiles/asebaswitch.dir/clean

switch/CMakeFiles/asebaswitch.dir/depend:
	cd /home/admin1/install/aseba/build-aseba && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/admin1/install/aseba/aseba /home/admin1/install/aseba/aseba/switch /home/admin1/install/aseba/build-aseba /home/admin1/install/aseba/build-aseba/switch /home/admin1/install/aseba/build-aseba/switch/CMakeFiles/asebaswitch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : switch/CMakeFiles/asebaswitch.dir/depend

