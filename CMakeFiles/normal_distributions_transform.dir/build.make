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
include CMakeFiles/normal_distributions_transform.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/normal_distributions_transform.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/normal_distributions_transform.dir/flags.make

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o: CMakeFiles/normal_distributions_transform.dir/flags.make
CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o: normal_distributions_transform.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bessa/Desktop/PESTI/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o -c /home/bessa/Desktop/PESTI/normal_distributions_transform.cpp

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bessa/Desktop/PESTI/normal_distributions_transform.cpp > CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.i

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bessa/Desktop/PESTI/normal_distributions_transform.cpp -o CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.s

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.requires:
.PHONY : CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.requires

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.provides: CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.requires
	$(MAKE) -f CMakeFiles/normal_distributions_transform.dir/build.make CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.provides.build
.PHONY : CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.provides

CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.provides.build: CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o

# Object files for target normal_distributions_transform
normal_distributions_transform_OBJECTS = \
"CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o"

# External object files for target normal_distributions_transform
normal_distributions_transform_EXTERNAL_OBJECTS =

normal_distributions_transform: CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o
normal_distributions_transform: /usr/lib/libboost_system-mt.so
normal_distributions_transform: /usr/lib/libboost_filesystem-mt.so
normal_distributions_transform: /usr/lib/libboost_thread-mt.so
normal_distributions_transform: /usr/lib/libboost_date_time-mt.so
normal_distributions_transform: /usr/lib/libboost_iostreams-mt.so
normal_distributions_transform: /usr/lib/libpcl_common.so
normal_distributions_transform: /usr/lib/libflann_cpp_s.a
normal_distributions_transform: /usr/lib/libpcl_kdtree.so
normal_distributions_transform: /usr/lib/libpcl_octree.so
normal_distributions_transform: /usr/lib/libpcl_search.so
normal_distributions_transform: /usr/lib/libpcl_sample_consensus.so
normal_distributions_transform: /usr/lib/libpcl_filters.so
normal_distributions_transform: /usr/lib/libOpenNI.so
normal_distributions_transform: /usr/lib/libvtkCommon.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkRendering.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkHybrid.so.5.8.0
normal_distributions_transform: /usr/lib/libpcl_io.so
normal_distributions_transform: /usr/lib/libpcl_features.so
normal_distributions_transform: /usr/lib/libpcl_keypoints.so
normal_distributions_transform: /usr/lib/libpcl_segmentation.so
normal_distributions_transform: /usr/lib/libpcl_visualization.so
normal_distributions_transform: /usr/lib/libpcl_tracking.so
normal_distributions_transform: /usr/lib/libqhull.so
normal_distributions_transform: /usr/lib/libpcl_surface.so
normal_distributions_transform: /usr/lib/libpcl_registration.so
normal_distributions_transform: /usr/lib/libpcl_apps.so
normal_distributions_transform: /usr/lib/libvtkParallel.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkRendering.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkGraphics.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkImaging.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkIO.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkFiltering.so.5.8.0
normal_distributions_transform: /usr/lib/libvtkCommon.so.5.8.0
normal_distributions_transform: /usr/lib/libvtksys.so.5.8.0
normal_distributions_transform: CMakeFiles/normal_distributions_transform.dir/build.make
normal_distributions_transform: CMakeFiles/normal_distributions_transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable normal_distributions_transform"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/normal_distributions_transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/normal_distributions_transform.dir/build: normal_distributions_transform
.PHONY : CMakeFiles/normal_distributions_transform.dir/build

CMakeFiles/normal_distributions_transform.dir/requires: CMakeFiles/normal_distributions_transform.dir/normal_distributions_transform.cpp.o.requires
.PHONY : CMakeFiles/normal_distributions_transform.dir/requires

CMakeFiles/normal_distributions_transform.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/normal_distributions_transform.dir/cmake_clean.cmake
.PHONY : CMakeFiles/normal_distributions_transform.dir/clean

CMakeFiles/normal_distributions_transform.dir/depend:
	cd /home/bessa/Desktop/PESTI && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI /home/bessa/Desktop/PESTI/CMakeFiles/normal_distributions_transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/normal_distributions_transform.dir/depend

