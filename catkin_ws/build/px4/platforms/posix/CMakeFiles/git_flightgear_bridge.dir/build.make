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

# Utility rule file for git_flightgear_bridge.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/git_flightgear_bridge.dir/progress.make

platforms/posix/CMakeFiles/git_flightgear_bridge: platforms/posix/git_init__home_rishabh_catkin_ws_src_PX4-Autopilot_Tools_flightgear_bridge.stamp


platforms/posix/git_init__home_rishabh_catkin_ws_src_PX4-Autopilot_Tools_flightgear_bridge.stamp: /home/rishabh/catkin_ws/src/PX4-Autopilot/.gitmodules
platforms/posix/git_init__home_rishabh_catkin_ws_src_PX4-Autopilot_Tools_flightgear_bridge.stamp: /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/flightgear_bridge/.git
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "git submodule Tools/flightgear_bridge"
	cd /home/rishabh/catkin_ws/src/PX4-Autopilot && Tools/check_submodules.sh Tools/flightgear_bridge
	cd /home/rishabh/catkin_ws/src/PX4-Autopilot && /usr/bin/cmake -E touch /home/rishabh/catkin_ws/build/px4/platforms/posix/git_init__home_rishabh_catkin_ws_src_PX4-Autopilot_Tools_flightgear_bridge.stamp

git_flightgear_bridge: platforms/posix/CMakeFiles/git_flightgear_bridge
git_flightgear_bridge: platforms/posix/git_init__home_rishabh_catkin_ws_src_PX4-Autopilot_Tools_flightgear_bridge.stamp
git_flightgear_bridge: platforms/posix/CMakeFiles/git_flightgear_bridge.dir/build.make

.PHONY : git_flightgear_bridge

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/git_flightgear_bridge.dir/build: git_flightgear_bridge

.PHONY : platforms/posix/CMakeFiles/git_flightgear_bridge.dir/build

platforms/posix/CMakeFiles/git_flightgear_bridge.dir/clean:
	cd /home/rishabh/catkin_ws/build/px4/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/git_flightgear_bridge.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/git_flightgear_bridge.dir/clean

platforms/posix/CMakeFiles/git_flightgear_bridge.dir/depend:
	cd /home/rishabh/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/src/PX4-Autopilot/platforms/posix /home/rishabh/catkin_ws/build/px4 /home/rishabh/catkin_ws/build/px4/platforms/posix /home/rishabh/catkin_ws/build/px4/platforms/posix/CMakeFiles/git_flightgear_bridge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/git_flightgear_bridge.dir/depend

