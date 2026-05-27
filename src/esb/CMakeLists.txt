#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
# Vendored from sdk-nrf v3.3.0 subsys/esb
# Lives inside zmk-softdevice-controller alongside the SDC/MPSL stack.

zephyr_library()
zephyr_library_sources(esb.c)
zephyr_library_sources(esb_workarounds.c)
zephyr_library_sources(esb_glue.c)

zephyr_library_sources_ifdef(CONFIG_HAS_HW_NRF_PPI esb_ppi.c)
zephyr_library_sources_ifndef(CONFIG_HAS_HW_NRF_PPI esb_dppi.c)

# esb.h lives alongside the sources
zephyr_include_directories(.)

# MPSL + FEM headers — already resolved by the parent sdc CMakeLists via NRFXLIB_DIR,
# but ESB compiles as its own zephyr_library so the include dirs don't propagate.
# Re-use the same NRFXLIB_DIR variable set by the parent.
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include/protocol)
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/include)

# multithreading_lock — sibling directory in sdc/mpsl/
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdc/mpsl)
