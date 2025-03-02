if(${CONFIG_NATIVE_MQTT})

    cmake_minimum_required(VERSION 3.11)

    # Include the FetchContent module
    include(FetchContent)

    # Define the content to fetch for Paho MQTT C library
    FetchContent_Declare(
  paho
  SOURCE_DIR ${ZEPHYR_PAHO_MQTT_C_MODULE_DIR}
)

    # Fetch and make the content available
    FetchContent_MakeAvailable(paho)

    # Set the required CMake arguments for the Paho MQTT C library
    set(PAHO_BUILD_STATIC TRUE)
    set(PAHO_BUILD_SHARED FALSE)
    set(PAHO_ENABLE_TESTING FALSE) # Disable building tests

    # Add the Paho MQTT C library as a CMake target
    # Generate a unique binary directory for the Paho MQTT C library
    set(PAHO_BINARY_DIR ${CMAKE_BINARY_DIR}/paho-build)

    # Add the Paho MQTT C library as a CMake target
    set_directory_properties(PROPERTIES EXCLUDE_FROM_ALL TRUE)
    add_subdirectory(${paho_SOURCE_DIR} ${PAHO_BINARY_DIR})

    # Create a Zephyr library
    zephyr_library()
    zephyr_library_property(ALLOW_EMPTY TRUE)
    zephyr_library_add_dependencies(paho-mqtt3c-static)

    # Add dependencies on the Paho MQTT C library
    zephyr_library_link_libraries(paho-mqtt3c-static)
    zephyr_link_libraries(paho-mqtt3c-static)

    # Add the include directories for Paho MQTT C library
    zephyr_include_directories(${paho_SOURCE_DIR}/src)


endif()
