# The mission data in data/*.json is converted to constant arrays at build
# time rather than parsed on the device. cJSON would build a whole DOM, and
# targets.json is large enough that the peak allocation is a real risk on a
# part with a few hundred KB of DRAM. The data never changes at runtime, so
# there is nothing to gain by paying for that.
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(MISSION_DATA "${GENERATED_DIR}/mission_data.generated.hpp")

# idf_component_register rejects an include directory that does not exist yet,
# and the generator has not run at configure time, so create it up front.
file(MAKE_DIRECTORY "${GENERATED_DIR}")

idf_component_register(
    SRCS
        "main.cpp"
        "sensor_uart.cpp"
        "seeker.cpp"
        "bay.cpp"
    INCLUDE_DIRS
        "."
        "${GENERATED_DIR}"
        "${REPO_ROOT}/include"
)

# ESP-IDF evaluates every component's CMakeLists twice: once in script mode to
# collect requirements, where add_custom_command does not exist, and once for
# real. Everything below belongs to the real pass only.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    idf_build_get_property(python PYTHON)

    add_custom_command(
        OUTPUT "${MISSION_DATA}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_DIR}"
        COMMAND ${python}
                "${REPO_ROOT}/tools/gen_mission_data.py"
                "${REPO_ROOT}/data/targets.json"
                "${REPO_ROOT}/data/ammo.json"
                "${MISSION_DATA}"
        DEPENDS "${REPO_ROOT}/tools/gen_mission_data.py"
                "${REPO_ROOT}/data/targets.json"
                "${REPO_ROOT}/data/ammo.json"
        COMMENT "Baking mission data into mission_data.generated.hpp"
        VERBATIM
    )
    add_custom_target(mission_data_gen DEPENDS "${MISSION_DATA}")

    # The generated header must exist before this component compiles.
    add_dependencies(${COMPONENT_LIB} mission_data_gen)
endif()
