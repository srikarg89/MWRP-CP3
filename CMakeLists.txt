cmake_minimum_required(VERSION 3.10)
project(STAPF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# Specify include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(stapf_lib src/watchman.cpp)

# Add an executable target
add_executable(run src/main.cpp)

target_link_libraries(run PRIVATE stapf_lib)
