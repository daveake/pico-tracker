cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(tracker_project)

pico_sdk_init()

add_executable(tracker
  tracker.c
  battery.c
  led.c
  gps.c
  bme280.c
  lora.c
  misc.c
  prediction.c
)

pico_add_extra_outputs(tracker)

target_link_libraries(tracker pico_stdlib hardware_spi hardware_i2c hardware_adc)


