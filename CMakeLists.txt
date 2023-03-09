project(lifelong)

cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 11)

set(USE_PYBIND FALSE)
IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()



include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


add_executable(lifelong ${SOURCES} )
target_link_libraries(lifelong ${Boost_LIBRARIES})

IF(USE_PYBIND)
    # replace 3.6 to the python version you use
    find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Interpreter Development)
    find_package(PythonInterp 3.6 EXACT REQUIRED)
    find_package(pybind11 REQUIRED)
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")

    pybind11_add_module(MAPF "./src/Grid.cpp"   "./src/States.cpp" "./python/MAPFbinding.cpp")
    target_link_libraries(MAPF PRIVATE ) 

    add_executable(py_driver "./src/Grid.cpp"  
                    "./src/States.cpp" 
                    "./src/common.cpp"
                    "./src/Validator.cpp"
                    "./src/ActionModel.cpp"
                    "./src/CompetitionSystem.cpp" 
                    "./src/MAPFPlanner.cpp"
                    "./python/pyMAPFPlanner.cpp" 
                    "./python/test.cpp" )
    target_link_libraries(py_driver PRIVATE pybind11::embed ${Boost_LIBRARIES})

ENDIF()




# Python Binding

