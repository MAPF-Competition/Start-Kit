project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

include_directories("inc")

# default planner
include_directories("default_planner")

file(GLOB SOURCES "src/*.cpp" "default_planner/*.cpp")
file(GLOB DEFAULT_PLANNER_SOURCES "default_planner/*.cpp")


# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)
include_directories( ${Boost_INCLUDE_DIRS} )

# pybind11 is a required dependency (needed for Python component support at runtime)
find_package(pybind11 REQUIRED)

# Python bridge headers
include_directories("python/common")

# Build the MAPF Python module (loadable .so for standalone Python scripts)
pybind11_add_module(MAPF "./src/Grid.cpp" "./src/States.cpp" "./python/common/MAPFbinding.cpp")
target_compile_definitions(MAPF PRIVATE MAPF_SHARED_MODULE)

# Collect bridge source files (include MAPFbinding.cpp for embedded module registration)
file(GLOB PY_BRIDGE_SOURCES "python/common/*.cpp")

add_executable(lifelong ${PY_BRIDGE_SOURCES} ${SOURCES})
target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})

if (COPY_PY_PATH_CONFIG)
    file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
endif()


add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)


set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so"
)


option(BUILD_TESTS "Build planner tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_executable(multi_step_plan_test
        tests/multi_step_plan_test.cpp
        ${DEFAULT_PLANNER_SOURCES}
    )
    target_link_libraries(multi_step_plan_test ${Boost_LIBRARIES})
    add_test(NAME multi_step_plan_test COMMAND multi_step_plan_test)

    add_executable(delay_generator_test
        tests/delay_generator_test.cpp
        src/DelayGenerator.cpp
        src/common.cpp
    )
    target_link_libraries(delay_generator_test ${Boost_LIBRARIES})
    add_test(NAME delay_generator_test COMMAND delay_generator_test)

    # Python interface test — needs all sources + bridge sources + pybind11::embed
    # Exclude driver.cpp (has its own main)
    set(TEST_SOURCES ${SOURCES})
    list(FILTER TEST_SOURCES EXCLUDE REGEX ".*driver\\.cpp")
    add_executable(python_interface_test
        tests/python_interface_test.cpp
        ${PY_BRIDGE_SOURCES}
        ${TEST_SOURCES}
    )
    target_link_libraries(python_interface_test PRIVATE pybind11::embed ${Boost_LIBRARIES})
    add_test(NAME python_interface_test COMMAND python_interface_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

