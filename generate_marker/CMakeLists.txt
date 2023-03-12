cmake_minimum_required(VERSION 3.1)
project(generate_marker)

set (CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR /home/user/build)
find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

set(example_src
    src/generate_marker.cpp
   )
add_executable(generate_marker ${example_src})
target_link_libraries(generate_marker
    ${OpenCV_LIBRARIES}
    )

target_compile_options(generate_marker
    PRIVATE -O3 -std=c++11
    )


