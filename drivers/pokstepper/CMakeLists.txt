# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_POKSTEPPER_ADI_TMC adi_tmc)
# zephyr-keep-sorted-stop

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)
zephyr_library_sources_ifdef(CONFIG_POKSTEPPER_POKIBOT_EMPTY_POKSTEPPER empty_pokstepper.c)
