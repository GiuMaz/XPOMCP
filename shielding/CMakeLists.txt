cmake_minimum_required(VERSION 3.9)
project(pomcp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost 1.33 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(pomcp
    ./src/battleship.cpp
    ./src/beliefstate.cpp
    ./src/coord.cpp
    ./src/experiment.cpp
    ./src/main.cpp
    ./src/mcts.cpp
    ./src/network.cpp
    ./src/node.cpp
    ./src/pocman.cpp
    ./src/rocksample.cpp
    ./src/simulator.cpp
    ./src/tag.cpp
    ./src/testsimulator.cpp
    ./src/utils.cpp
    ./src/tiger.cpp
    ./src/obstacleavoidance.cpp
    )
target_include_directories(pomcp PUBLIC ./src)
target_link_libraries(pomcp LINK_PUBLIC ${Boost_LIBRARIES})
