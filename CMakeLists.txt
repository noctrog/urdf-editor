cmake_minimum_required(VERSION 3.10)


project(urdf-editor)

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
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90
)
FetchContent_Declare(
  nfd
  GIT_REPOSITORY https://github.com/btzy/nativefiledialog-extended.git
  GIT_TAG v1.1.1
)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 10.2.1
)
# set(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
# set(ASSIMP_BUILD_TESTS OFF)
# set(ASSIMP_INSTALL OFF)
FetchContent_Declare(
  assimp
  GIT_REPOSITORY https://github.com/assimp/assimp.git
  GIT_TAG v5.3.1
)
set(LOGURU_WITH_STREAMS TRUE)
FetchContent_MakeAvailable(raylib imgui pugixml loguru nfd fmt assimp)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${raylib_SOURCE_DIR}/src/external/glfw/include
)

target_compile_definitions(imgui PUBLIC IMGUI_DEFINE_MATH_OPERATORS)

file(GLOB SRC_FILES "src/*.cc")
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PRIVATE raylib imgui pugixml loguru nfd fmt assimp)

target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wfloat-conversion -pedantic)
