cmake_minimum_required(VERSION 3.10.0)
project(STAPF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(CPLEX_ROOT_DIR /opt/ibm/ILOG/CPLEX_Studio2211/)

include(cmake/CPM.cmake)

find_package(nlohmann_json 3.11.0 REQUIRED) # Specify required version
find_package(Boost 1.70 REQUIRED COMPONENTS system filesystem program_options)
find_package(Cplex REQUIRED)

CPMAddPackage(
  NAME bshoshany-thread-pool
  GITHUB_REPOSITORY bshoshany/thread-pool
  GIT_TAG v5.0.0 # Or the latest stable release tag
)


set(CONCORDE_DIR "/home/srikar/libraries/concorde_build")

# Specify include directories for the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CONCORDE_DIR})
include_directories(/home/srikar/libraries/qsopt)
include_directories(${CPLEX_INCLUDE_DIRS})
include_directories(${bshoshany-thread-pool_SOURCE_DIR}/include)

add_library(stapf_lib src/search.cpp)

# Add an executable target
add_executable(run src/main.cpp)
add_executable(pruning_experiment src/pruning_experiment.cpp)

target_link_libraries(run PRIVATE stapf_lib nlohmann_json::nlohmann_json Boost::system Boost::filesystem Boost::program_options)
target_link_libraries(run PRIVATE ${CPLEX_LIBRARIES} ${CONCORDE_DIR}/concorde.a /home/srikar/libraries/qsopt/lib/qsopt.a)
# target_compile_options(run PRIVATE -Wno-deprecated-declarations) # For GCC/Clang

target_link_libraries(pruning_experiment PRIVATE stapf_lib nlohmann_json::nlohmann_json Boost::system Boost::filesystem Boost::program_options)
target_link_libraries(pruning_experiment PRIVATE ${CPLEX_LIBRARIES} ${CONCORDE_DIR}/concorde.a /home/srikar/libraries/qsopt/lib/qsopt.a)
