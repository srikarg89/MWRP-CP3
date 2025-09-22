cmake_minimum_required(VERSION 3.10)
project(STAPF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(nlohmann_json 3.11.0 REQUIRED) # Specify required version
find_package(Boost 1.70 REQUIRED COMPONENTS system filesystem program_options)

# Specify include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})

add_library(stapf_lib src/watchman.cpp)

# Add an executable target
add_executable(run src/main.cpp)

target_link_libraries(run PRIVATE stapf_lib nlohmann_json::nlohmann_json Boost::system Boost::filesystem Boost::program_options)
