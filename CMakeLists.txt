cmake_minimum_required(VERSION 3.10)
project(CasadiExample)

# C++17 사용
set(CMAKE_CXX_STANDARD 17)


# CasADi 경로 수동 설정
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# 실행 파일 추가
add_executable(main main.cpp)
# add_executable(srb SRB.cpp)


# CasADi 라이브러리 연결
target_link_libraries(main PRIVATE casadi)

