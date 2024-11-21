# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  TEMPLATE_3_1_6
  VERSION 0.1
  DESCRIPTION "Template for MuJoCo(3.1.6)"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set (SOURCES
    src/main.cpp
    
    src/simulate.cpp
    src/glfw_adapter.cpp
    src/glfw_dispatch.cpp
    src/lodepng.cpp
    src/platform_ui_adapter.cpp

    src/trajectory.cpp
    src/kinematics.cpp
    src/filter.cpp
    src/controller.cpp
)

add_executable (main ${SOURCES})

target_include_directories(main PUBLIC
    ${CMAKE_SOURCE_DIR}/includes
    ${CMAKE_SOURCE_DIR}/../../include
    ${CMAKE_SOURCE_DIR}/../../include/mujoco
)
link_directories(main          #Library directory
    ${CMAKE_SOURCE_DIR}/../../lib
)

target_link_libraries(main
    mujoco
    glfw
)
