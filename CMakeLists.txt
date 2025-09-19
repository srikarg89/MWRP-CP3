cmake_minimum_required(VERSION 3.10)
project(MyProject LANGUAGES CXX)

# Specify include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add an executable target
add_executable(run src/main.cpp)
