
cmake_minimum_required(VERSION 3.10)
project(qpOASES3)

set(QPOASES_INCLUDE_DIR /usr/local/include)
set(QPOASES_LIB_DIR /usr/local/lib)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES 
            src/main.cpp
            # src/example2.cpp
            # src/SRB_oneleg.cpp
            
            
            src/BaseDataSaver.cpp
            src/ExtendedDataSaver.cpp
)


add_executable(main ${SOURCES})

target_include_directories(main PUBLIC
    ${QPOASES_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include)


link_directories(${QPOASES_LIB_DIR})

target_link_libraries(main PUBLIC
    qpOASES
    )
