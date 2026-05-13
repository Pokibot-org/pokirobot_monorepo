cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pokirobot_brain_soft)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
    src/main.c
    src/nav/nav.c
    # src/control/control.c
    src/pokarm/pokarm.c
    src/pomicontrol/pomicontrol.c
    src/astar-jps/IndexPriorityQueue.c
    src/astar-jps/AStar.c
)


if(BOARD STREQUAL "native_sim")
target_sources(app PRIVATE
    src/const_publisher.c
    src/control/control_sim.c
)
else()
target_sources(app PRIVATE
    src/control/control.c
)
endif()

# Quick hack for Clion to include all zephyr workspace files in the project file tree view.
include_directories(${ZEPHYR_BASE}/..)
