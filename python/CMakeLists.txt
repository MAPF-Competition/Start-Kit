cmake_minimum_required(VERSION 3.16)
project(mapf_pybind)


find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Interpreter Development)
find_package(PythonInterp 3.6 EXACT REQUIRED)
find_package(pybind11 REQUIRED)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ version selection")

# find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
# include_directories( ${Boost_INCLUDE_DIRS} )


IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")


include_directories("../inc")
pybind11_add_module(MAPF "../src/Grid.cpp"   "../src/States.cpp" "./MAPFbinding.cpp")
target_link_libraries(MAPF PRIVATE ) 




