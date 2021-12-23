cmake_minimum_required(VERSION 3.9)
project(pomcp CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(Z3_HOME CACHE STRING "Z3 installation directory")
find_package(Z3)
if (Z3_FOUND)
  add_definitions(-DWITH_Z3)
  include_directories(${Z3_INCLUDE_DIR})
endif()
#message( ${Z3_VERSION_STRING} )

find_package(Boost 1.33 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_executable(pomcp
    ./src/main.cpp

    ./src/battleship.cpp
    ./src/battery_velocity.cpp
    ./src/beliefstate.cpp
    ./src/coord.cpp
    ./src/experiment.cpp
    ./src/mcts.cpp
    ./src/network.cpp
    ./src/node.cpp
    ./src/pocman.cpp
    ./src/rocksample.cpp
    ./src/refuel.cpp
    ./src/simulator.cpp
    ./src/tag.cpp
    ./src/lasertag.cpp
    ./src/testsimulator.cpp
    ./src/utils.cpp
    ./src/tiger.cpp
    ./src/obstacleavoidance.cpp
    ./src/xes_logger.cpp
    ./src/node_lazy.cpp
    ./src/simulator_lazy.cpp
    ./src/mcts_lazy.cpp
    ./src/experiment_lazy.cpp
    )
target_include_directories(pomcp PUBLIC ./src)
target_link_libraries(pomcp LINK_PUBLIC ${Boost_LIBRARIES})

target_include_directories(pomcp PRIVATE ${Z3_INCLUDE_DIRS})
target_link_libraries(pomcp PRIVATE ${Z3_LIBRARIES})

add_executable(xpomcp
    ./src/xpomcp.cpp

    ./src/battleship.cpp
    ./src/battery_velocity.cpp
    ./src/beliefstate.cpp
    ./src/coord.cpp
    ./src/experiment.cpp
    ./src/trace.h
    ./src/mcts.cpp
    ./src/network.cpp
    ./src/node.cpp
    ./src/pocman.cpp
    ./src/rocksample.cpp
    ./src/refuel.cpp
    ./src/rocksample_trace.cpp
    ./src/obstacleavoidance_trace.cpp
    ./src/simulator.cpp
    ./src/tag.cpp
    ./src/lasertag.cpp
    ./src/testsimulator.cpp
    ./src/utils.cpp
    ./src/tiger.cpp
    ./src/obstacleavoidance.cpp
    ./src/xes_logger.cpp
    ./src/node_lazy.cpp
    ./src/simulator_lazy.cpp
    ./src/mcts_lazy.cpp
    ./src/experiment_lazy.cpp
    )
target_include_directories(xpomcp PUBLIC ./src)
target_link_libraries(xpomcp LINK_PUBLIC ${Boost_LIBRARIES})

target_include_directories(xpomcp PRIVATE ${Z3_INCLUDE_DIRS})
target_link_libraries(xpomcp PRIVATE ${Z3_LIBRARIES})


add_executable(xpomcp_vr
    ./src/xpomcp_vr.cpp

    ./src/battleship.cpp
    ./src/battery_velocity.cpp
    ./src/beliefstate.cpp
    ./src/coord.cpp
    ./src/experiment.cpp
    ./src/trace.h
    ./src/mcts.cpp
    ./src/network.cpp
    ./src/node.cpp
    ./src/pocman.cpp
    ./src/rocksample.cpp
    ./src/refuel.cpp
    ./src/rocksample_trace.cpp
    ./src/obstacleavoidance_trace.cpp
    ./src/obstacleavoidance_trace.cpp
    ./src/simulator.cpp
    ./src/tag.cpp
    ./src/lasertag.cpp
    ./src/testsimulator.cpp
    ./src/utils.cpp
    ./src/tiger.cpp
    ./src/obstacleavoidance.cpp
    ./src/xes_logger.cpp
    ./src/node_lazy.cpp
    ./src/simulator_lazy.cpp
    ./src/mcts_lazy.cpp
    ./src/experiment_lazy.cpp
    )
target_include_directories(xpomcp_vr PUBLIC ./src)
target_link_libraries(xpomcp_vr LINK_PUBLIC ${Boost_LIBRARIES})

target_include_directories(xpomcp_vr PRIVATE ${Z3_INCLUDE_DIRS})
target_link_libraries(xpomcp_vr PRIVATE ${Z3_LIBRARIES})
