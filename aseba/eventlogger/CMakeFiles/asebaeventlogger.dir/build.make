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
include eventlogger/CMakeFiles/asebaeventlogger.dir/depend.make

# Include the progress variables for this target.
include eventlogger/CMakeFiles/asebaeventlogger.dir/progress.make

# Include the compile flags for this target's objects.
include eventlogger/CMakeFiles/asebaeventlogger.dir/flags.make

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o: eventlogger/CMakeFiles/asebaeventlogger.dir/flags.make
eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o: /home/admin1/install/aseba/aseba/eventlogger/eventlogger.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o"
	cd /home/admin1/install/aseba/build-aseba/eventlogger && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o -c /home/admin1/install/aseba/aseba/eventlogger/eventlogger.cpp

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.i"
	cd /home/admin1/install/aseba/build-aseba/eventlogger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/eventlogger/eventlogger.cpp > CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.i

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.s"
	cd /home/admin1/install/aseba/build-aseba/eventlogger && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/eventlogger/eventlogger.cpp -o CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.s

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.requires:
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.requires

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.provides: eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.requires
	$(MAKE) -f eventlogger/CMakeFiles/asebaeventlogger.dir/build.make eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.provides.build
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.provides

eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.provides.build: eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o

# Object files for target asebaeventlogger
asebaeventlogger_OBJECTS = \
"CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o"

# External object files for target asebaeventlogger
asebaeventlogger_EXTERNAL_OBJECTS =

eventlogger/asebaeventlogger: eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o
eventlogger/asebaeventlogger: eventlogger/CMakeFiles/asebaeventlogger.dir/build.make
eventlogger/asebaeventlogger: libasebacore.a
eventlogger/asebaeventlogger: /usr/lib/libqwt-qt4.so
eventlogger/asebaeventlogger: /usr/lib/i386-linux-gnu/libQtGui.so
eventlogger/asebaeventlogger: /usr/lib/i386-linux-gnu/libQtCore.so
eventlogger/asebaeventlogger: /home/admin1/install/aseba/build-dashel/libdashel.a
eventlogger/asebaeventlogger: /usr/lib/i386-linux-gnu/libudev.so
eventlogger/asebaeventlogger: /home/admin1/install/aseba/build-dashel/libdashel.a
eventlogger/asebaeventlogger: /usr/lib/i386-linux-gnu/libudev.so
eventlogger/asebaeventlogger: eventlogger/CMakeFiles/asebaeventlogger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable asebaeventlogger"
	cd /home/admin1/install/aseba/build-aseba/eventlogger && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/asebaeventlogger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
eventlogger/CMakeFiles/asebaeventlogger.dir/build: eventlogger/asebaeventlogger
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/build

eventlogger/CMakeFiles/asebaeventlogger.dir/requires: eventlogger/CMakeFiles/asebaeventlogger.dir/eventlogger.cpp.o.requires
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/requires

eventlogger/CMakeFiles/asebaeventlogger.dir/clean:
	cd /home/admin1/install/aseba/build-aseba/eventlogger && $(CMAKE_COMMAND) -P CMakeFiles/asebaeventlogger.dir/cmake_clean.cmake
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/clean

eventlogger/CMakeFiles/asebaeventlogger.dir/depend:
	cd /home/admin1/install/aseba/build-aseba && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/admin1/install/aseba/aseba /home/admin1/install/aseba/aseba/eventlogger /home/admin1/install/aseba/build-aseba /home/admin1/install/aseba/build-aseba/eventlogger /home/admin1/install/aseba/build-aseba/eventlogger/CMakeFiles/asebaeventlogger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : eventlogger/CMakeFiles/asebaeventlogger.dir/depend

