# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_POKLEGSCOM_IMPL_POKMAC poklegscom_pokmac.c)
zephyr_library_sources_ifdef(CONFIG_POKLEGSCOM_IMPL_SIMULATED poklegscom_msm.c)
