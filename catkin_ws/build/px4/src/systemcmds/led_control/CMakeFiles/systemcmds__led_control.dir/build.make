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
CMAKE_SOURCE_DIR = /home/rishabh/catkin_ws/src/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rishabh/catkin_ws/build/px4

# Include any dependencies generated for this target.
include src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/flags.make

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/flags.make
src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o: /home/rishabh/catkin_ws/src/PX4-Autopilot/src/systemcmds/led_control/led_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o"
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && /usr/bin/ccache /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o -c /home/rishabh/catkin_ws/src/PX4-Autopilot/src/systemcmds/led_control/led_control.cpp

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/systemcmds__led_control.dir/led_control.cpp.i"
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rishabh/catkin_ws/src/PX4-Autopilot/src/systemcmds/led_control/led_control.cpp > CMakeFiles/systemcmds__led_control.dir/led_control.cpp.i

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/systemcmds__led_control.dir/led_control.cpp.s"
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rishabh/catkin_ws/src/PX4-Autopilot/src/systemcmds/led_control/led_control.cpp -o CMakeFiles/systemcmds__led_control.dir/led_control.cpp.s

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.requires:

.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.requires

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.provides: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.requires
	$(MAKE) -f src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/build.make src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.provides.build
.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.provides

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.provides.build: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o


# Object files for target systemcmds__led_control
systemcmds__led_control_OBJECTS = \
"CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o"

# External object files for target systemcmds__led_control
systemcmds__led_control_EXTERNAL_OBJECTS =

/home/rishabh/catkin_ws/devel/.private/px4/lib/libsystemcmds__led_control.a: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o
/home/rishabh/catkin_ws/devel/.private/px4/lib/libsystemcmds__led_control.a: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/build.make
/home/rishabh/catkin_ws/devel/.private/px4/lib/libsystemcmds__led_control.a: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/rishabh/catkin_ws/devel/.private/px4/lib/libsystemcmds__led_control.a"
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__led_control.dir/cmake_clean_target.cmake
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__led_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/build: /home/rishabh/catkin_ws/devel/.private/px4/lib/libsystemcmds__led_control.a

.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/build

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/requires: src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/led_control.cpp.o.requires

.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/requires

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/clean:
	cd /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__led_control.dir/cmake_clean.cmake
.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/clean

src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/depend:
	cd /home/rishabh/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/src/PX4-Autopilot/src/systemcmds/led_control /home/rishabh/catkin_ws/build/px4 /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control /home/rishabh/catkin_ws/build/px4/src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/led_control/CMakeFiles/systemcmds__led_control.dir/depend

