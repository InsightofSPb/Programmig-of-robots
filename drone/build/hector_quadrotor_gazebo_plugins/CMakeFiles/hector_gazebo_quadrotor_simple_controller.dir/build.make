# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alexander/drone/build/hector_quadrotor_gazebo_plugins

# Include any dependencies generated for this target.
include CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/flags.make

CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o: CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/flags.make
CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o: /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins/src/gazebo_quadrotor_simple_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alexander/drone/build/hector_quadrotor_gazebo_plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o -c /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins/src/gazebo_quadrotor_simple_controller.cpp

CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins/src/gazebo_quadrotor_simple_controller.cpp > CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.i

CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins/src/gazebo_quadrotor_simple_controller.cpp -o CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.s

# Object files for target hector_gazebo_quadrotor_simple_controller
hector_gazebo_quadrotor_simple_controller_OBJECTS = \
"CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o"

# External object files for target hector_gazebo_quadrotor_simple_controller
hector_gazebo_quadrotor_simple_controller_EXTERNAL_OBJECTS =

/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/src/gazebo_quadrotor_simple_controller.cpp.o
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/build.make
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.2
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libtf.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libtf2_ros.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libactionlib.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libtf2.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /home/alexander/drone/devel/.private/hector_quadrotor_model/lib/libhector_quadrotor_propulsion.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /home/alexander/drone/devel/.private/hector_quadrotor_model/lib/libhector_quadrotor_aerodynamics.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libroscpp.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/librosconsole.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/librostime.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/libcpp_common.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/x86_64-linux-gnu/libfcl.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/liboctomap.so.1.9.8
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /opt/ros/noetic/lib/liboctomath.so.1.9.8
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.3.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.6.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.10.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.13.0
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.2
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so: CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alexander/drone/build/hector_quadrotor_gazebo_plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/build: /home/alexander/drone/devel/.private/hector_quadrotor_gazebo_plugins/lib/libhector_gazebo_quadrotor_simple_controller.so

.PHONY : CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/build

CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/clean

CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/depend:
	cd /home/alexander/drone/build/hector_quadrotor_gazebo_plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins /home/alexander/drone/src/hector_quadrotor/hector_quadrotor_gazebo_plugins /home/alexander/drone/build/hector_quadrotor_gazebo_plugins /home/alexander/drone/build/hector_quadrotor_gazebo_plugins /home/alexander/drone/build/hector_quadrotor_gazebo_plugins/CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hector_gazebo_quadrotor_simple_controller.dir/depend

