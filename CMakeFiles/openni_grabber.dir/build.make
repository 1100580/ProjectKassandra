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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bessa/Desktop/PESTI

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bessa/Desktop/PESTI

# Include any dependencies generated for this target.
include CMakeFiles/openni_grabber.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openni_grabber.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openni_grabber.dir/flags.make

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o: CMakeFiles/openni_grabber.dir/flags.make
CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o: openni_grabber.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bessa/Desktop/PESTI/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o -c /home/bessa/Desktop/PESTI/openni_grabber.cpp

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni_grabber.dir/openni_grabber.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bessa/Desktop/PESTI/openni_grabber.cpp > CMakeFiles/openni_grabber.dir/openni_grabber.cpp.i

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni_grabber.dir/openni_grabber.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bessa/Desktop/PESTI/openni_grabber.cpp -o CMakeFiles/openni_grabber.dir/openni_grabber.cpp.s

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.requires:
.PHONY : CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.requires

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.provides: CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.requires
	$(MAKE) -f CMakeFiles/openni_grabber.dir/build.make CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.provides.build
.PHONY : CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.provides

CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.provides.build: CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o

# Object files for target openni_grabber
openni_grabber_OBJECTS = \
"CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o"

# External object files for target openni_grabber
openni_grabber_EXTERNAL_OBJECTS =

openni_grabber: CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o
openni_grabber: /usr/lib/libboost_system-mt.so
openni_grabber: /usr/lib/libboost_filesystem-mt.so
openni_grabber: /usr/lib/libboost_thread-mt.so
openni_grabber: /usr/lib/libboost_date_time-mt.so
openni_grabber: /usr/lib/libboost_iostreams-mt.so
openni_grabber: /usr/lib/libpcl_common.so
openni_grabber: /usr/lib/libflann_cpp_s.a
openni_grabber: /usr/lib/libpcl_kdtree.so
openni_grabber: /usr/lib/libpcl_octree.so
openni_grabber: /usr/lib/libpcl_search.so
openni_grabber: /usr/lib/libpcl_sample_consensus.so
openni_grabber: /usr/lib/libpcl_filters.so
openni_grabber: /usr/lib/libOpenNI.so
openni_grabber: /usr/lib/libvtkCommon.so.5.8.0
openni_grabber: /usr/lib/libvtkRendering.so.5.8.0
openni_grabber: /usr/lib/libvtkHybrid.so.5.8.0
openni_grabber: /usr/lib/libpcl_io.so
openni_grabber: /usr/lib/libpcl_features.so
openni_grabber: /usr/lib/libpcl_keypoints.so
openni_grabber: /usr/lib/libpcl_segmentation.so
openni_grabber: /usr/lib/libpcl_visualization.so
openni_grabber: /usr/lib/libpcl_tracking.so
openni_grabber: /usr/lib/libqhull.so
openni_grabber: /usr/lib/libpcl_surface.so
openni_grabber: /usr/lib/libpcl_registration.so
openni_grabber: /usr/lib/libpcl_apps.so
openni_grabber: /usr/lib/libvtkParallel.so.5.8.0
openni_grabber: /usr/lib/libvtkRendering.so.5.8.0
openni_grabber: /usr/lib/libvtkGraphics.so.5.8.0
openni_grabber: /usr/lib/libvtkImaging.so.5.8.0
openni_grabber: /usr/lib/libvtkIO.so.5.8.0
openni_grabber: /usr/lib/libvtkFiltering.so.5.8.0
openni_grabber: /usr/lib/libvtkCommon.so.5.8.0
openni_grabber: /usr/lib/libvtksys.so.5.8.0
openni_grabber: CMakeFiles/openni_grabber.dir/build.make
openni_grabber: CMakeFiles/openni_grabber.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable openni_grabber"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni_grabber.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openni_grabber.dir/build: openni_grabber
.PHONY : CMakeFiles/openni_grabber.dir/build

CMakeFiles/openni_grabber.dir/requires: CMakeFiles/openni_grabber.dir/openni_grabber.cpp.o.requires
.PHONY : CMakeFiles/openni_grabber.dir/requires

CMakeFiles/openni_grabber.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openni_grabber.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openni_grabber.dir/clean

CMakeFiles/openni_grabber.dir/depend:
	cd /home/bessa/Desktop/PESTI && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI/CMakeFiles/openni_grabber.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openni_grabber.dir/depend
