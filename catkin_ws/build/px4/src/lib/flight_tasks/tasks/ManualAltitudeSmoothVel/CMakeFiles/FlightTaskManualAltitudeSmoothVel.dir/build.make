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
include src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/depend.make

# Include the progress variables for this target.
include src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/flags.make

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/flags.make
src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o: /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/FlightTaskManualAltitudeSmoothVel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o"
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && /usr/bin/ccache /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o -c /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/FlightTaskManualAltitudeSmoothVel.cpp

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.i"
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/FlightTaskManualAltitudeSmoothVel.cpp > CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.i

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.s"
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/FlightTaskManualAltitudeSmoothVel.cpp -o CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.s

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.requires:

.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.requires

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.provides: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.requires
	$(MAKE) -f src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/build.make src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.provides.build
.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.provides

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.provides.build: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o


# Object files for target FlightTaskManualAltitudeSmoothVel
FlightTaskManualAltitudeSmoothVel_OBJECTS = \
"CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o"

# External object files for target FlightTaskManualAltitudeSmoothVel
FlightTaskManualAltitudeSmoothVel_EXTERNAL_OBJECTS =

/home/rishabh/catkin_ws/devel/.private/px4/lib/libFlightTaskManualAltitudeSmoothVel.a: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o
/home/rishabh/catkin_ws/devel/.private/px4/lib/libFlightTaskManualAltitudeSmoothVel.a: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/build.make
/home/rishabh/catkin_ws/devel/.private/px4/lib/libFlightTaskManualAltitudeSmoothVel.a: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rishabh/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/rishabh/catkin_ws/devel/.private/px4/lib/libFlightTaskManualAltitudeSmoothVel.a"
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && $(CMAKE_COMMAND) -P CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/cmake_clean_target.cmake
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/build: /home/rishabh/catkin_ws/devel/.private/px4/lib/libFlightTaskManualAltitudeSmoothVel.a

.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/build

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/requires: src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/FlightTaskManualAltitudeSmoothVel.cpp.o.requires

.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/requires

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/clean:
	cd /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel && $(CMAKE_COMMAND) -P CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/cmake_clean.cmake
.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/clean

src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/depend:
	cd /home/rishabh/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rishabh/catkin_ws/src/PX4-Autopilot /home/rishabh/catkin_ws/src/PX4-Autopilot/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel /home/rishabh/catkin_ws/build/px4 /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel /home/rishabh/catkin_ws/build/px4/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/CMakeFiles/FlightTaskManualAltitudeSmoothVel.dir/depend

