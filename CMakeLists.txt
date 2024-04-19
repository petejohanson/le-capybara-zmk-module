add_subdirectory(drivers)

if (CONFIG_SLEEP_AWAKE_TRIGGER)
    target_sources(app PRIVATE src/sleep_awake_trigger.c)
    target_include_directories(app PRIVATE include)
endif()