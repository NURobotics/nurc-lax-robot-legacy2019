# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/nvidia/nurc-lax-robot/MainEvent

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/nurc-lax-robot/MainEvent/build

# Include any dependencies generated for this target.
include CMakeFiles/cv_nurc_lax.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cv_nurc_lax.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cv_nurc_lax.dir/flags.make

CMakeFiles/cv_nurc_lax.dir/main.cpp.o: CMakeFiles/cv_nurc_lax.dir/flags.make
CMakeFiles/cv_nurc_lax.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/nurc-lax-robot/MainEvent/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cv_nurc_lax.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_nurc_lax.dir/main.cpp.o -c /home/nvidia/nurc-lax-robot/MainEvent/main.cpp

CMakeFiles/cv_nurc_lax.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_nurc_lax.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/nurc-lax-robot/MainEvent/main.cpp > CMakeFiles/cv_nurc_lax.dir/main.cpp.i

CMakeFiles/cv_nurc_lax.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_nurc_lax.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/nurc-lax-robot/MainEvent/main.cpp -o CMakeFiles/cv_nurc_lax.dir/main.cpp.s

CMakeFiles/cv_nurc_lax.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/cv_nurc_lax.dir/main.cpp.o.requires

CMakeFiles/cv_nurc_lax.dir/main.cpp.o.provides: CMakeFiles/cv_nurc_lax.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/cv_nurc_lax.dir/build.make CMakeFiles/cv_nurc_lax.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/cv_nurc_lax.dir/main.cpp.o.provides

CMakeFiles/cv_nurc_lax.dir/main.cpp.o.provides.build: CMakeFiles/cv_nurc_lax.dir/main.cpp.o


# Object files for target cv_nurc_lax
cv_nurc_lax_OBJECTS = \
"CMakeFiles/cv_nurc_lax.dir/main.cpp.o"

# External object files for target cv_nurc_lax
cv_nurc_lax_EXTERNAL_OBJECTS =

cv_nurc_lax: CMakeFiles/cv_nurc_lax.dir/main.cpp.o
cv_nurc_lax: CMakeFiles/cv_nurc_lax.dir/build.make
cv_nurc_lax: /usr/local/lib/libopencv_stitching.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_shape.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudacodec.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_img_hash.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_line_descriptor.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_face.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudaobjdetect.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudabgsegm.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudalegacy.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_ccalib.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_surface_matching.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudastereo.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_reg.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_datasets.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_aruco.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_structured_light.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_bgsegm.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_rgbd.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudafeatures2d.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_videostab.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cvv.so.4.1.0
cv_nurc_lax: ../ML/libdarknet.so
cv_nurc_lax: /usr/local/lib/libopencv_objdetect.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_text.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_ml.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_dnn.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_photo.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_video.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_calib3d.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudawarping.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudaimgproc.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudafilters.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudaarithm.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_features2d.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_flann.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_highgui.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_videoio.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_imgproc.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_core.so.4.1.0
cv_nurc_lax: /usr/local/lib/libopencv_cudev.so.4.1.0
cv_nurc_lax: CMakeFiles/cv_nurc_lax.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/nurc-lax-robot/MainEvent/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cv_nurc_lax"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cv_nurc_lax.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cv_nurc_lax.dir/build: cv_nurc_lax

.PHONY : CMakeFiles/cv_nurc_lax.dir/build

CMakeFiles/cv_nurc_lax.dir/requires: CMakeFiles/cv_nurc_lax.dir/main.cpp.o.requires

.PHONY : CMakeFiles/cv_nurc_lax.dir/requires

CMakeFiles/cv_nurc_lax.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cv_nurc_lax.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cv_nurc_lax.dir/clean

CMakeFiles/cv_nurc_lax.dir/depend:
	cd /home/nvidia/nurc-lax-robot/MainEvent/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/nurc-lax-robot/MainEvent /home/nvidia/nurc-lax-robot/MainEvent /home/nvidia/nurc-lax-robot/MainEvent/build /home/nvidia/nurc-lax-robot/MainEvent/build /home/nvidia/nurc-lax-robot/MainEvent/build/CMakeFiles/cv_nurc_lax.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cv_nurc_lax.dir/depend

