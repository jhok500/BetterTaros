cmake_minimum_required(VERSION 2.8.12)
project(omega_algorithm)

set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

if(CMAKE_CURRENT_BINARY_DIR MATCHES generated)
    EXEC_PROGRAM(rm ARGS -rf ${CMAKE_SOURCE_DIR}/build)
    EXEC_PROGRAM(ln ARGS -s ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/build)
endif(CMAKE_CURRENT_BINARY_DIR MATCHES generated)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Find the GSL package
find_package(GSL REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

# Descend into the controllers directory
add_subdirectory(controllers)
