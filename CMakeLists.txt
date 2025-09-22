cmake_minimum_required(VERSION 3.10)
project(STAPF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(nlohmann_json 3.11.0 REQUIRED) # Specify required version
find_package(Boost 1.70 REQUIRED COMPONENTS system filesystem program_options)

set(CONCORDE_DIR "/home/srikar/libraries/concorde_build")

# Specify include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CONCORDE_DIR})
include_directories(/home/srikar/libraries/qsopt)

add_library(stapf_lib src/watchman.cpp)

# Add an executable target
add_executable(run src/main.cpp)


target_link_libraries(run PRIVATE stapf_lib nlohmann_json::nlohmann_json Boost::system Boost::filesystem Boost::program_options ${CONCORDE_DIR}/concorde.a /home/srikar/libraries/qsopt/lib/qsopt.a)
