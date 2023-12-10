cmake_minimum_required(VERSION 3.10)


project(ray-urdf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.0
)
FetchContent_Declare(
  pugixml
  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG v1.14
)
FetchContent_Declare(
  loguru
  GIT_REPOSITORY https://github.com/emilk/loguru.git
  GIT_TAG 4adaa185883e3c04da25913579c451d3c32cfac1
)
set(LOGURU_WITH_STREAMS TRUE)
FetchContent_MakeAvailable(raylib pugixml loguru)

file(GLOB SRC_FILES "src/*.cc")
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PRIVATE raylib pugixml loguru)

target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wfloat-conversion -pedantic)
