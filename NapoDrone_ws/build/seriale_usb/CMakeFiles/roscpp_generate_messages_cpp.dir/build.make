# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build

# Utility rule file for roscpp_generate_messages_cpp.

# Include the progress variables for this target.
include seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/progress.make

roscpp_generate_messages_cpp: seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/build.make
.PHONY : roscpp_generate_messages_cpp

# Rule to build all files generated by this target.
seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/build: roscpp_generate_messages_cpp
.PHONY : seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/build

seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/clean:
	cd /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/seriale_usb && $(CMAKE_COMMAND) -P CMakeFiles/roscpp_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/clean

seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/depend:
	cd /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/src/seriale_usb /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/seriale_usb /home/stefano/Progetto_Drone/Odroid_ArCaRa/NapoDrone_ws/build/seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : seriale_usb/CMakeFiles/roscpp_generate_messages_cpp.dir/depend

