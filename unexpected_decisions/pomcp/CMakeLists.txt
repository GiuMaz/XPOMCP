cmake_minimum_required(VERSION 3.9)
project(pomcp CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost 1.33 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(pomcp
    ./src/beliefstate.cpp
    ./src/experiment.cpp
    ./src/main.cpp
    ./src/mcts.cpp
    ./src/network.cpp
    ./src/node.cpp
    ./src/obstacleavoidance.cpp
    ./src/simulator.cpp
    ./src/testsimulator.cpp
    ./src/tiger.cpp
    ./src/utils.cpp
    )
target_include_directories(pomcp PUBLIC ./src/core)
target_link_libraries(pomcp LINK_PUBLIC ${Boost_LIBRARIES})
