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

# Utility rule file for gazebo_iris_obs_avoid_ddd_warehouse.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/progress.make

platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse:
	cd /home/rishabh/catkin_ws/build/px4/tmp && /home/rishabh/catkin_ws/src/PX4-Autopilot/Tools/sitl_run.sh /home/rishabh/catkin_ws/devel/.private/px4/lib/px4/px4 ddd gazebo iris_obs_avoid warehouse /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/build/px4

gazebo_iris_obs_avoid_ddd_warehouse: platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse
gazebo_iris_obs_avoid_ddd_warehouse: platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/build.make

.PHONY : gazebo_iris_obs_avoid_ddd_warehouse

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/build: gazebo_iris_obs_avoid_ddd_warehouse

.PHONY : platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/build

platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/clean:
	cd /home/rishabh/catkin_ws/build/px4/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/clean

platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/depend:
	cd /home/rishabh/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/src/PX4-Autopilot/platforms/posix /home/rishabh/catkin_ws/build/px4 /home/rishabh/catkin_ws/build/px4/platforms/posix /home/rishabh/catkin_ws/build/px4/platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/gazebo_iris_obs_avoid_ddd_warehouse.dir/depend
