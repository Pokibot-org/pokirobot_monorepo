cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pokirobot_legs_soft)

target_include_directories(app PRIVATE src)

file(GLOB_RECURSE APP_SOURCES src/*.c)

target_sources(app PRIVATE ${APP_SOURCES})
