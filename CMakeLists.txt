# ---------- nrfxlib discovery (shared by SDC + ESB) ----------
# Check module directory first, then workspace root.
get_filename_component(_MODULE_ROOT ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(NRFXLIB_DIR ${_MODULE_ROOT}/nrfxlib)
if(NOT EXISTS ${NRFXLIB_DIR})
  get_filename_component(_WS_ROOT ${ZEPHYR_BASE}/.. ABSOLUTE)
  if(EXISTS ${_WS_ROOT}/nrfxlib)
    set(NRFXLIB_DIR ${_WS_ROOT}/nrfxlib)
  endif()
endif()

if(CONFIG_ZMK_BT_LL_SOFTDEVICE)
  add_subdirectory(src/sdc)
endif()

# ESB driver (vendored from sdk-nrf v3.3.0) — shares MPSL/nrfxlib with SDC.
# Works with or without BLE (MPSL timeslot mode when BLE active, exclusive otherwise).
if(CONFIG_ESB)
  if(NOT EXISTS ${NRFXLIB_DIR})
    message(FATAL_ERROR "nrfxlib not found (needed by ESB). Run 'west update'.")
  endif()
  add_subdirectory(src/esb)
endif()

# ull_vendor.h must be on the include path for the controller's ull.c when
# USER_EXT is enabled (not just ESB_TICKER_TIMESLOT).
if(CONFIG_BT_CTLR_USER_EXT)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/src/esb)

  # HCI user ext stubs — always needed when USER_EXT is enabled.
  # When CONFIG_ESB is on, src/esb/CMakeLists.txt handles compilation.
  # Only compile from here when ESB is off (isolation / fallback).
  if(NOT CONFIG_ESB)
    zephyr_library_named(zmk_hci_user_ext)
    zephyr_library_sources(src/esb/hci_user_ext.c)
    zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller)
    zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/hci)
    zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/ll_sw)
    zephyr_include_directories(${ZEPHYR_BASE}/subsys/bluetooth/controller/include)
  endif()
endif()

if(CONFIG_ZMK_BT_LL_SOFTDEVICE AND CONFIG_BT_SUBRATING)
  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()

if(CONFIG_ZMK_BT_LL_SOFTDEVICE AND CONFIG_BT_SUBRATING AND CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_library_sources(src/subrating.c)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
endif()

if(CONFIG_ZMK_BLE_QOS)
  zephyr_library_sources(src/qos.c)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/src/util/chmap_filter/include)

  if(CONFIG_FPU)
    set(CHMAP_FLOAT_ABI "hard-float")
  else()
    set(CHMAP_FLOAT_ABI "soft-float")
  endif()

  zephyr_link_libraries(
    ${CMAKE_CURRENT_LIST_DIR}/src/util/chmap_filter/lib/cortex-m4/${CHMAP_FLOAT_ABI}/libchmapfilt.a
  )
endif()

if(CONFIG_ZMK_BLE_POWER_CONTROL_AUTO)
  zephyr_library_sources(src/power_control.c)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
endif()

if(CONFIG_ZMK_BLE_SPLIT_LINK_TX_POWER_SET)
  zephyr_library_sources(src/split_tx_power.c)
endif()

if(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
  zephyr_library_sources(src/path_loss.c)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
endif()
