# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)

# Name the project
project(FluidSimulation)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify the path to the SDL2 library
set(SDL2_DIR ${CMAKE_SOURCE_DIR}/libs/SDL2)

# Include the SDL2 include directory
include_directories(${SDL2_DIR}/include)

# Link the SDL2 library directory
link_directories(${SDL2_DIR}/lib)

# Add the source files
file(GLOB SOURCES src/*.cpp)

# Create the executable
add_executable(FluidSimulation ${SOURCES})

# Link the executable with SDL2main and SDL2
target_link_libraries(FluidSimulation SDL2main SDL2)

# Copy SDL2.dll to the build output directory on Windows
if (WIN32)
    add_custom_command(TARGET FluidSimulation POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${SDL2_DIR}/lib/SDL2.dll"
        $<TARGET_FILE_DIR:FluidSimulation>)
endif()
