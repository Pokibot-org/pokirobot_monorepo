# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SHARED_UART          shared_uart)
add_subdirectory_ifdef(CONFIG_DELTA_UPDATE         delta_update)
add_subdirectory_ifdef(CONFIG_SIMULATION           simulation)
add_subdirectory_ifdef(CONFIG_MQTT_SIM_MESSAGER    mqtt_sim_messager)
