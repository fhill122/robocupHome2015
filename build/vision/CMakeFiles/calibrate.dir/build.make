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
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocuphome/robocuphome2015/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocuphome/robocuphome2015/build

# Include any dependencies generated for this target.
include vision/CMakeFiles/calibrate.dir/depend.make

# Include the progress variables for this target.
include vision/CMakeFiles/calibrate.dir/progress.make

# Include the compile flags for this target's objects.
include vision/CMakeFiles/calibrate.dir/flags.make

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o: vision/CMakeFiles/calibrate.dir/flags.make
vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o: /home/robocuphome/robocuphome2015/src/vision/src/calibrate.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robocuphome/robocuphome2015/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o"
	cd /home/robocuphome/robocuphome2015/build/vision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/calibrate.dir/src/calibrate.cpp.o -c /home/robocuphome/robocuphome2015/src/vision/src/calibrate.cpp

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibrate.dir/src/calibrate.cpp.i"
	cd /home/robocuphome/robocuphome2015/build/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robocuphome/robocuphome2015/src/vision/src/calibrate.cpp > CMakeFiles/calibrate.dir/src/calibrate.cpp.i

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibrate.dir/src/calibrate.cpp.s"
	cd /home/robocuphome/robocuphome2015/build/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robocuphome/robocuphome2015/src/vision/src/calibrate.cpp -o CMakeFiles/calibrate.dir/src/calibrate.cpp.s

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.requires:
.PHONY : vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.requires

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.provides: vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.requires
	$(MAKE) -f vision/CMakeFiles/calibrate.dir/build.make vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.provides.build
.PHONY : vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.provides

vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.provides.build: vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o

# Object files for target calibrate
calibrate_OBJECTS = \
"CMakeFiles/calibrate.dir/src/calibrate.cpp.o"

# External object files for target calibrate
calibrate_EXTERNAL_OBJECTS =

/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_videostab.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_video.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_superres.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_stitching.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_photo.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_objdetect.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_nonfree.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_ml.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_legacy.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_imgproc.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_highgui.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_gpu.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_flann.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_features2d.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_core.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_contrib.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_calib3d.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_nonfree.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_gpu.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_photo.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_objdetect.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_legacy.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_video.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_ml.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_calib3d.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_features2d.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_highgui.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_imgproc.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_flann.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: /opt/ros/groovy/lib/libopencv_core.so.2.4.9
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: vision/CMakeFiles/calibrate.dir/build.make
/home/robocuphome/robocuphome2015/devel/lib/vision/calibrate: vision/CMakeFiles/calibrate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/robocuphome/robocuphome2015/devel/lib/vision/calibrate"
	cd /home/robocuphome/robocuphome2015/build/vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibrate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/CMakeFiles/calibrate.dir/build: /home/robocuphome/robocuphome2015/devel/lib/vision/calibrate
.PHONY : vision/CMakeFiles/calibrate.dir/build

vision/CMakeFiles/calibrate.dir/requires: vision/CMakeFiles/calibrate.dir/src/calibrate.cpp.o.requires
.PHONY : vision/CMakeFiles/calibrate.dir/requires

vision/CMakeFiles/calibrate.dir/clean:
	cd /home/robocuphome/robocuphome2015/build/vision && $(CMAKE_COMMAND) -P CMakeFiles/calibrate.dir/cmake_clean.cmake
.PHONY : vision/CMakeFiles/calibrate.dir/clean

vision/CMakeFiles/calibrate.dir/depend:
	cd /home/robocuphome/robocuphome2015/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocuphome/robocuphome2015/src /home/robocuphome/robocuphome2015/src/vision /home/robocuphome/robocuphome2015/build /home/robocuphome/robocuphome2015/build/vision /home/robocuphome/robocuphome2015/build/vision/CMakeFiles/calibrate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/CMakeFiles/calibrate.dir/depend
