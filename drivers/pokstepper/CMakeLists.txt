# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_POKSTEPPER_ADI_TMC adi_tmc)
# zephyr-keep-sorted-stop

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)
