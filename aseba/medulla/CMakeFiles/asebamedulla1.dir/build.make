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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/epfl/Downloads/aseba/aseba

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/epfl/Downloads/aseba/build-aseba

# Include any dependencies generated for this target.
include medulla/CMakeFiles/asebamedulla1.dir/depend.make

# Include the progress variables for this target.
include medulla/CMakeFiles/asebamedulla1.dir/progress.make

# Include the compile flags for this target's objects.
include medulla/CMakeFiles/asebamedulla1.dir/flags.make

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o: medulla/CMakeFiles/asebamedulla1.dir/flags.make
medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o: /home/epfl/Downloads/aseba/aseba/medulla/medulla3.cpp
medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o: medulla/medulla.moc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/epfl/Downloads/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o"
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebamedulla1.dir/medulla3.cpp.o -c /home/epfl/Downloads/aseba/aseba/medulla/medulla3.cpp

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebamedulla1.dir/medulla3.cpp.i"
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/epfl/Downloads/aseba/aseba/medulla/medulla3.cpp > CMakeFiles/asebamedulla1.dir/medulla3.cpp.i

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebamedulla1.dir/medulla3.cpp.s"
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/epfl/Downloads/aseba/aseba/medulla/medulla3.cpp -o CMakeFiles/asebamedulla1.dir/medulla3.cpp.s

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.requires:
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.requires

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.provides: medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.requires
	$(MAKE) -f medulla/CMakeFiles/asebamedulla1.dir/build.make medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.provides.build
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.provides

medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.provides.build: medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o

medulla/medulla.moc: /home/epfl/Downloads/aseba/aseba/medulla/medulla.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/epfl/Downloads/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating medulla.moc"
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && /usr/bin/moc-qt4 -I/home/epfl/Downloads/aseba/dashel -I/usr/include -I/usr/include/qt4 -I/usr/include/qt4/QtDBus -I/usr/include/qt4/QtXml -I/usr/include/qt4/QtCore -DASEBA_ASSERT -DQT_DBUS_LIB -DQT_XML_LIB -DQT_CORE_LIB -o /home/epfl/Downloads/aseba/build-aseba/medulla/medulla.moc /home/epfl/Downloads/aseba/aseba/medulla/medulla.h

# Object files for target asebamedulla1
asebamedulla1_OBJECTS = \
"CMakeFiles/asebamedulla1.dir/medulla3.cpp.o"

# External object files for target asebamedulla1
asebamedulla1_EXTERNAL_OBJECTS =

medulla/asebamedulla1: medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o
medulla/asebamedulla1: medulla/CMakeFiles/asebamedulla1.dir/build.make
medulla/asebamedulla1: medulla/CMakeFiles/asebamedulla1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable asebamedulla1"
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/asebamedulla1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
medulla/CMakeFiles/asebamedulla1.dir/build: medulla/asebamedulla1
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/build

medulla/CMakeFiles/asebamedulla1.dir/requires: medulla/CMakeFiles/asebamedulla1.dir/medulla3.cpp.o.requires
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/requires

medulla/CMakeFiles/asebamedulla1.dir/clean:
	cd /home/epfl/Downloads/aseba/build-aseba/medulla && $(CMAKE_COMMAND) -P CMakeFiles/asebamedulla1.dir/cmake_clean.cmake
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/clean

medulla/CMakeFiles/asebamedulla1.dir/depend: medulla/medulla.moc
	cd /home/epfl/Downloads/aseba/build-aseba && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/epfl/Downloads/aseba/aseba /home/epfl/Downloads/aseba/aseba/medulla /home/epfl/Downloads/aseba/build-aseba /home/epfl/Downloads/aseba/build-aseba/medulla /home/epfl/Downloads/aseba/build-aseba/medulla/CMakeFiles/asebamedulla1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : medulla/CMakeFiles/asebamedulla1.dir/depend

