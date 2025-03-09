# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_POKUICOM_IMPL_POKMAC pokuicom_pokmac.c)
zephyr_library_sources_ifdef(CONFIG_POKUICOM_IMPL_SIMULATED pokuicom_msm.c)
