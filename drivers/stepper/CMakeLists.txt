# SPDX-FileCopyrightText: Copyright (c) 2023 Carl Zeiss Meditec AG
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/stepper.h)

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_STEPPER_ADI_TMC adi_tmc)
# zephyr-keep-sorted-stop

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)
