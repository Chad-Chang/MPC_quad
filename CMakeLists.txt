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
set(QPOASES_INCLUDE_DIR /usr/local/include)
set(QPOASES_LIB_DIR /usr/local/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set (SOURCES
    src/main.cpp
    
    src/logging/BaseDataSaver.cpp
    src/logging/ExtendedDataSaver.cpp

    src/ui/simulate.cpp
    src/ui/glfw_adapter.cpp
    src/ui/glfw_dispatch.cpp
    src/ui/lodepng.cpp
    src/ui/platform_ui_adapter.cpp

    src/optimization/optimizer.cpp

    src/mcl_controller/trajectory.cpp
    src/mcl_controller/kinematics.cpp
    src/mcl_controller/filter.cpp
    src/mcl_controller/controller.cpp
)
find_package(PkgConfig REQUIRED)
# qpOASES 포함 경로 추가
include_directories(${QPOASES_INCLUDE_DIRS})



add_executable (main ${SOURCES})

target_include_directories(main PUBLIC
    ${QPOASES_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/includes
    ${CMAKE_SOURCE_DIR}/includes/logging
    ${CMAKE_SOURCE_DIR}/includes/optimization
    ${CMAKE_SOURCE_DIR}/includes/mcl_controller
    ${CMAKE_SOURCE_DIR}/includes/ui
    ${CMAKE_SOURCE_DIR}/../../include
    ${CMAKE_SOURCE_DIR}/../../include/mujoco
)
link_directories(main          #Library directory
    ${CMAKE_SOURCE_DIR}/../../lib
    ${QPOASES_LIB_DIR}
)

target_link_libraries(main
    mujoco
    glfw
    ${QPOASES_LIBRARIES}
    qpOASES
)


# 컴파일러 플래그 추가 (필요 시)
#target_compile_options(main PRIVATE ${QPOASES_CFLAGS_OTHER})
