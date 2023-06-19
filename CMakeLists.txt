project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 11)

# Set the default value of PYTHON to false
option(PYTHON "Enable Python" OFF)


# Print the value of PYTHON
message(STATUS "PYTHON: ${PYTHON}")



IF(NOT CMAKE_BUILD_TYPE)
    # SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")

# Find Boost
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
#SET(Boost_USE_STATIC_LIBS ON)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)

include_directories( ${Boost_INCLUDE_DIRS} )


IF(PYTHON)
    message(STATUS "Python support is enabled")
    find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Interpreter Development)
    find_package(PythonInterp 3.6  REQUIRED)
    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")


    pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE ) 

    set(PY_SOURCES "./src/Grid.cpp"  
    "./src/States.cpp" 
    "./src/common.cpp"
    "./src/Validator.cpp"
    "./src/ActionModel.cpp"
    "./src/CompetitionSystem.cpp" 
    "./src/MAPFPlanner.cpp"
    "./src/Logger.cpp"
    "./python/pyMAPFPlanner.cpp" 
    "./python/py_driver.cpp"
    "./src/Evaluation.cpp")


    add_executable(lifelong  ${PY_SOURCES})
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES})

ELSE()
    message(STATUS "Python support is disabled")
    
    file(GLOB SOURCES "src/*.cpp")




    add_executable(lifelong ${SOURCES} )
    target_link_libraries(lifelong ${Boost_LIBRARIES})
ENDIF()




# IF(USE_PYBIND)
    # replace 3.6 to the python version you use






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

# ENDIF()




# Python Binding

