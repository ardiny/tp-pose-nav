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
include CMakeFiles/asebacompiler.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/asebacompiler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/asebacompiler.dir/flags.make

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o: /home/admin1/install/aseba/aseba/compiler/compiler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o -c /home/admin1/install/aseba/aseba/compiler/compiler.cpp

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/compiler.cpp > CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.i

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/compiler.cpp -o CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.s

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o: /home/admin1/install/aseba/aseba/compiler/errors.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o -c /home/admin1/install/aseba/aseba/compiler/errors.cpp

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/errors.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/errors.cpp > CMakeFiles/asebacompiler.dir/compiler/errors.cpp.i

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/errors.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/errors.cpp -o CMakeFiles/asebacompiler.dir/compiler/errors.cpp.s

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o: /home/admin1/install/aseba/aseba/compiler/identifier-lookup.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o -c /home/admin1/install/aseba/aseba/compiler/identifier-lookup.cpp

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/identifier-lookup.cpp > CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.i

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/identifier-lookup.cpp -o CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.s

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o: /home/admin1/install/aseba/aseba/compiler/lexer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o -c /home/admin1/install/aseba/aseba/compiler/lexer.cpp

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/lexer.cpp > CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.i

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/lexer.cpp -o CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.s

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o: /home/admin1/install/aseba/aseba/compiler/parser.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o -c /home/admin1/install/aseba/aseba/compiler/parser.cpp

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/parser.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/parser.cpp > CMakeFiles/asebacompiler.dir/compiler/parser.cpp.i

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/parser.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/parser.cpp -o CMakeFiles/asebacompiler.dir/compiler/parser.cpp.s

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o: /home/admin1/install/aseba/aseba/compiler/analysis.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o -c /home/admin1/install/aseba/aseba/compiler/analysis.cpp

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/analysis.cpp > CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.i

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/analysis.cpp -o CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.s

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-build.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-build.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-build.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-build.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-expand.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-expand.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-expand.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-expand.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-dump.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-dump.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-dump.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-dump.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-typecheck.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-typecheck.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-typecheck.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-typecheck.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-optimize.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-optimize.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-optimize.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-optimize.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o: CMakeFiles/asebacompiler.dir/flags.make
CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o: /home/admin1/install/aseba/aseba/compiler/tree-emit.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/admin1/install/aseba/build-aseba/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o -c /home/admin1/install/aseba/aseba/compiler/tree-emit.cpp

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/admin1/install/aseba/aseba/compiler/tree-emit.cpp > CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.i

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/admin1/install/aseba/aseba/compiler/tree-emit.cpp -o CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.s

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.requires:
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.requires

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.provides: CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.requires
	$(MAKE) -f CMakeFiles/asebacompiler.dir/build.make CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.provides.build
.PHONY : CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.provides

CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.provides.build: CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o

# Object files for target asebacompiler
asebacompiler_OBJECTS = \
"CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o" \
"CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o"

# External object files for target asebacompiler
asebacompiler_EXTERNAL_OBJECTS =

libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o
libasebacompiler.a: CMakeFiles/asebacompiler.dir/build.make
libasebacompiler.a: CMakeFiles/asebacompiler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libasebacompiler.a"
	$(CMAKE_COMMAND) -P CMakeFiles/asebacompiler.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/asebacompiler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/asebacompiler.dir/build: libasebacompiler.a
.PHONY : CMakeFiles/asebacompiler.dir/build

CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/compiler.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/errors.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/identifier-lookup.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/lexer.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/parser.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/analysis.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-build.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-expand.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-dump.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-typecheck.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-optimize.cpp.o.requires
CMakeFiles/asebacompiler.dir/requires: CMakeFiles/asebacompiler.dir/compiler/tree-emit.cpp.o.requires
.PHONY : CMakeFiles/asebacompiler.dir/requires

CMakeFiles/asebacompiler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/asebacompiler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/asebacompiler.dir/clean

CMakeFiles/asebacompiler.dir/depend:
	cd /home/admin1/install/aseba/build-aseba && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/admin1/install/aseba/aseba /home/admin1/install/aseba/aseba /home/admin1/install/aseba/build-aseba /home/admin1/install/aseba/build-aseba /home/admin1/install/aseba/build-aseba/CMakeFiles/asebacompiler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/asebacompiler.dir/depend
