cmake_minimum_required(VERSION 3.20)

project(testing CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

include(FetchContent)

FetchContent_Declare(
    json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(json)

add_library(droneBallistics_Thread
    src/AnalyticalSolver.cpp
    src/BallisticTable.cpp
    src/TableSolver.cpp
    src/ComponentsFabric.cpp
    src/FileConfigLoader.cpp
    src/JsonTargetProvider.cpp
    src/MissionProcessor.cpp
    src/states/States.cpp
    src/mt/DronePhysics.cpp
    src/mt/ThreadSafeTargetProvider.cpp
    src/mt/DroneStates.cpp
    src/mt/MissionRunner.cpp
)
target_include_directories(droneBallistics_Thread PUBLIC include)
target_link_libraries(droneBallistics_Thread PUBLIC nlohmann_json::nlohmann_json)

find_package(Threads REQUIRED)
target_link_libraries(droneBallistics_Thread PUBLIC Threads::Threads)

add_executable(droneBallistics_Thread_mt src/main_mt.cpp)
target_link_libraries(droneBallistics_Thread_mt PRIVATE droneBallistics_Thread)
# --- tests ---------------------------------------------------------------
# The sensor link is the seam between the ESP32 and the Pi, so its framing is
# worth testing on the host where failures are cheap to find. The header is
# standalone, so this needs neither the simulator library nor a board.
add_executable(sensorLinkTest tests/SensorLinkTest.cpp)
target_include_directories(sensorLinkTest PRIVATE include)
add_test(NAME sensor_link COMMAND sensorLinkTest)
