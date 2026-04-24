cmake_minimum_required(VERSION 3.16)
project(ungula_loadcell_lib_tests)

set(CMAKE_CXX_STANDARD 17)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# ---------------------------------------------------------------------------
# Dependencies: UngulaCore (TimeControl stub hooks).
#
# By default we use sibling directory ../../lib which is the layout during
# local development. External consumers can set -DUSE_LOCAL_DEPS=OFF to fetch
# it from GitHub instead.
# ---------------------------------------------------------------------------
option(USE_LOCAL_DEPS "Use local sibling lib/ directory" ON)

if(USE_LOCAL_DEPS)
    set(LIB_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib/src)
    set(LIB_HAL_SRC ${CMAKE_CURRENT_LIST_DIR}/../../lib_hal/src)
else()
    FetchContent_Declare(
        ungula_core
        GIT_REPOSITORY https://github.com/alexconesap/ungula-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(ungula_core)
    set(LIB_SRC ${ungula_core_SOURCE_DIR}/src)
endif()

set(LIB_LOADCELL_SRC ${CMAKE_CURRENT_LIST_DIR}/../src)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/ArduinoStub
    ${LIB_SRC}
    ${LIB_HAL_SRC}
    ${LIB_LOADCELL_SRC}
)

# ---------------------------------------------------------------------------
# LoadCell tests — pure host tests, no hardware. LoadCell depends only on
# IAdc24, so MockAdc24 gives full coverage of calibration, offset, unit
# conversion and read delegation without a real ADC.
# ---------------------------------------------------------------------------
add_executable(test_ungula_loadcell
    test_load_cell.cpp

    ${LIB_LOADCELL_SRC}/loadcell/load_cell.cpp
)
target_link_libraries(test_ungula_loadcell gtest_main)

# ---------------------------------------------------------------------------
# Driver compile + IAdc24 contract tests. Exercises all four concrete drivers
# through the desktop HAL stubs — verifies they compile, link, and honour the
# IAdc24 interface without real hardware.
# ---------------------------------------------------------------------------
add_executable(test_ungula_drivers
    test_drivers.cpp

    ${LIB_LOADCELL_SRC}/loadcell/drivers/hx711.cpp
    ${LIB_LOADCELL_SRC}/loadcell/drivers/nau7802.cpp
    ${LIB_LOADCELL_SRC}/loadcell/drivers/ads1220.cpp
    ${LIB_LOADCELL_SRC}/loadcell/drivers/ads1232.cpp
    ${LIB_HAL_SRC}/hal/i2c/platforms/i2c_master_default.cpp
    ${LIB_HAL_SRC}/hal/spi/platforms/spi_master_default.cpp
)
target_link_libraries(test_ungula_drivers gtest_main)

# ---------------------------------------------------------------------------
# TensionSensor tests — uses MockAdc24 + LoadCell. Covers filtering, stability
# detection, target tracking and readStable timeout behaviour.
# ---------------------------------------------------------------------------
add_executable(test_ungula_tension
    test_tension_sensor.cpp

    ${LIB_LOADCELL_SRC}/loadcell/load_cell.cpp
    ${LIB_LOADCELL_SRC}/loadcell/tension_sensor.cpp
)
target_link_libraries(test_ungula_tension gtest_main)

# ---------------------------------------------------------------------------
# Force conversion and safe tension calculator tests — header-only, no deps.
# ---------------------------------------------------------------------------
add_executable(test_ungula_force_convert
    test_force_convert.cpp
)
target_link_libraries(test_ungula_force_convert gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(test_ungula_loadcell)
gtest_discover_tests(test_ungula_drivers)
gtest_discover_tests(test_ungula_tension)
gtest_discover_tests(test_ungula_force_convert)
