project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)

if(PYTHON)
    set(PYTHON_FLAG 1)
else()
    set(PYTHON_FLAG 0)
endif()


# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")

IF(NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")

# default planner
include_directories("default_planner")

file(GLOB SOURCES "src/*.cpp" "default_planner/*.cpp")


# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)

include_directories( ${Boost_INCLUDE_DIRS} )


IF(PYTHON)
    message(STATUS "Python support is enabled")

    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")

     # check if python/tmp exists, raise an error if it does not
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/python/tmp")
        message(FATAL_ERROR "Track is not configured for python bindings. Please run python/set_track.bash <combined|planner|scheduler> to configure the track before compiling.")
    endif()

    include_directories("python/tmp")

    pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/tmp/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE ) 

    file(GLOB PY_SOURCES "python/tmp/*.cpp")
    list(FILTER PY_SOURCES EXCLUDE REGEX ".*python/MAPFbinding\.cpp")

    add_executable(lifelong  ${PY_SOURCES} ${SOURCES})
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

    if (COPY_PY_PATH_CONFIG)
        file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
    endif()


ELSE()
    message(STATUS "Python support is disabled")
    

    add_executable(lifelong ${SOURCES} )
    target_link_libraries(lifelong ${Boost_LIBRARIES})
    target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})
ENDIF()


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



