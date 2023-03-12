cmake_minimum_required(VERSION 3.1)
project(generate_board)

set (CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR /home/user/build)
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

set(example_src
    src/generate_board.cpp
   )
add_executable(generate_board ${example_src})
target_link_libraries(generate_board
    ${OpenCV_LIBRARIES}
    )

target_compile_options(generate_board
    PRIVATE -O3 -std=c++11
    )


