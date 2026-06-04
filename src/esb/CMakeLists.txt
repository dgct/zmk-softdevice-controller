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

# HCI user ext stubs needed whenever USER_EXT is enabled
if(CONFIG_BT_CTLR_USER_EXT)
  zephyr_library_sources(hci_user_ext.c)
endif()

# ESB ticker node: ULL/LLL integration with open BLE LL
if(CONFIG_ESB_TICKER_TIMESLOT)
  zephyr_library_sources(ull_esb.c)
  zephyr_library_sources(lll_esb.c)

  # ull_vendor.h must be on the include path for the BLE controller's ull.c
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../include)

  # BLE controller internal headers needed by ULL/LLL modules
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/lll)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ticker)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/include)
  zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/hci)
endif()

# esb.h lives alongside the sources
zephyr_include_directories(.)

# MPSL + FEM headers — already resolved by the parent sdc CMakeLists via NRFXLIB_DIR,
# but ESB compiles as its own zephyr_library so the include dirs don't propagate.
# Re-use the same NRFXLIB_DIR variable set by the parent.
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include)
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include/protocol)
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/include)

# multithreading_lock — sibling directory in sdc/mpsl/
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sdc/mpsl)

# When building without BLE/SDC, the MPSL FEM common library isn't linked by the
# sdc CMakeLists. ESB calls FEM APIs unconditionally, so link the stubs directly.
if(NOT CONFIG_ZMK_BT_LL_SOFTDEVICE)
  if(CONFIG_SOC_NRF52840)
    set(_ESB_SOC_FAMILY "nrf52")
  endif()
  if(CONFIG_FPU)
    set(_ESB_FLOAT_ABI "hard-float")
  else()
    set(_ESB_FLOAT_ABI "soft-float")
  endif()
  zephyr_link_libraries(${NRFXLIB_DIR}/mpsl/fem/common/lib/${_ESB_SOC_FAMILY}/${_ESB_FLOAT_ABI}/libmpsl_fem_common.a)
endif()
