
if (CONFIG_ZMK_ADXL362_AWAKE_TRIGGER)

zephyr_library()
zephyr_library_sources(adxl362_awake_trigger.c)

endif()
