# ESP32 sensor module firmware.
#
# The onboard peripheral half of the thesis: this board plays the seeker and
# the payload bay. It reports the targets it observes and the store it
# carries over UART, and takes no commands. The Raspberry Pi runs the flight
# computer and consumes those reports in place of reading data/targets.json
# and the ammo entry from data/ammo.json.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_drone_publisher)
