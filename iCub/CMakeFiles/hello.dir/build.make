# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub"

# Include any dependencies generated for this target.
include CMakeFiles/hello.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hello.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hello.dir/flags.make

CMakeFiles/hello.dir/main.o: CMakeFiles/hello.dir/flags.make
CMakeFiles/hello.dir/main.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hello.dir/main.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hello.dir/main.o -c "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/main.cpp"

CMakeFiles/hello.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello.dir/main.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/main.cpp" > CMakeFiles/hello.dir/main.i

CMakeFiles/hello.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello.dir/main.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/main.cpp" -o CMakeFiles/hello.dir/main.s

CMakeFiles/hello.dir/Robot.o: CMakeFiles/hello.dir/flags.make
CMakeFiles/hello.dir/Robot.o: Robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/hello.dir/Robot.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hello.dir/Robot.o -c "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/Robot.cpp"

CMakeFiles/hello.dir/Robot.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello.dir/Robot.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/Robot.cpp" > CMakeFiles/hello.dir/Robot.i

CMakeFiles/hello.dir/Robot.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello.dir/Robot.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/Robot.cpp" -o CMakeFiles/hello.dir/Robot.s

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o: CMakeFiles/hello.dir/flags.make
CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o: /mnt/c/Users/Gonçalo\ Cunha/Desktop/Tese/thesis/utils/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o -c "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/utils/utils.cpp"

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/utils/utils.cpp" > CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.i

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/utils/utils.cpp" -o CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.s

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o: CMakeFiles/hello.dir/flags.make
CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o: /mnt/c/Users/Gonçalo\ Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o -c "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.cpp"

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.cpp" > CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.i

CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.cpp" -o CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.s

# Object files for target hello
hello_OBJECTS = \
"CMakeFiles/hello.dir/main.o" \
"CMakeFiles/hello.dir/Robot.o" \
"CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o" \
"CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o"

# External object files for target hello
hello_EXTERNAL_OBJECTS =

hello: CMakeFiles/hello.dir/main.o
hello: CMakeFiles/hello.dir/Robot.o
hello: CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/utils/utils.o
hello: CMakeFiles/hello.dir/mnt/c/Users/Gonçalo_Cunha/Desktop/Tese/thesis/iCubWorld/simobjloader.o
hello: CMakeFiles/hello.dir/build.make
hello: /lib/libYARP_init.so.3.3.103
hello: /lib/libYARP_dev.so.3.3.103
hello: /lib/libYARP_math.so.3.3.103
hello: /lib/libYARP_gsl.so.3.3.103
hello: /lib/libYARP_sig.so.3.3.103
hello: /lib/libYARP_os.so.3.3.103
hello: CMakeFiles/hello.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable hello"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hello.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hello.dir/build: hello

.PHONY : CMakeFiles/hello.dir/build

CMakeFiles/hello.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hello.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hello.dir/clean

CMakeFiles/hello.dir/depend:
	cd "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub" "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub" "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub" "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub" "/mnt/c/Users/Gonçalo Cunha/Desktop/Tese/thesis/iCub/CMakeFiles/hello.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/hello.dir/depend

