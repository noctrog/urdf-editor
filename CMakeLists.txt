cmake_minimum_required(VERSION 3.11) # Set the minimum version of CMake.


project(ray-urdf) # Replace with your project name.

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(LibXml2 REQUIRED)

# Fetch raylib from GitHub.
include(FetchContent)
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.0 # Replace with the specific version or branch.
)
FetchContent_MakeAvailable(raylib)

include_directories(${LIBXML2_INCLUDE_DIR})

# Add your source files here.
add_executable(${PROJECT_NAME} main.c) # Replace 'main.c' with your source files.

# Link raylib to your project.
target_link_libraries(${PROJECT_NAME} raylib ${LIBXML2_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wfloat-conversion -pedantic)
