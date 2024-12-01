cmake_minimum_required(VERSION 3.10)
project(qpOASES_test)

set(QPOASES_INCLUDE_DIR /usr/local/include)
set(QPOASES_LIB_DIR /usr/local/lib)

include_directories(${QPOASES_INCLUDE_DIR})
link_directories(${QPOASES_LIB_DIR})

add_executable(main main.cpp)
target_link_libraries(main qpOASES)
