if(CONFIG_ZMK_BT_LL_SOFTDEVICE)
  add_subdirectory(src/sdc)
endif()

if(CONFIG_ZMK_BT_LL_SOFTDEVICE AND CONFIG_BT_SUBRATING)
  zephyr_library_sources(src/subrating.c)
  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()

if(CONFIG_ZMK_BLE_LLPM)
  zephyr_library_sources(src/llpm.c)
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
