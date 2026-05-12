cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_pokstepper)

target_include_directories(app PRIVATE src)

file(GLOB_RECURSE APP_SOURCES src/*.c)

target_sources(app PRIVATE ${APP_SOURCES})

# Quick hack for Clion to include all zephyr workspace files in the project file tree view.
include_directories(${ZEPHYR_BASE}/..)
