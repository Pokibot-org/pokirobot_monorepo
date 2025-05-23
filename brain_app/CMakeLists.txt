cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pokirobot_brain_soft)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
    src/main.c
    src/nav/nav.c
    src/pokarm/pokarm.c
    src/pakicontrol/pakicontrol.c
    src/astar-jps/IndexPriorityQueue.c
    src/astar-jps/AStar.c
)

if (CONFIG_MQTT_SIM_MESSENGER)
    target_sources(app PRIVATE
    src/const_publisher.c
)
endif()
