project(libMCNP2CAD)

cmake_minimum_required(VERSION 3.0)

set(MCNP2CAD_VERSION_MAJOR 1)
set(MCNP2CAD_VERSION_MINOR 0)
set(MCNP2CAD_VERSION_PATCH 5)

# Build options
option(STANDALONE_MCNP2CAD  "Build mcnp2cad as a standalone program" ON)
option(BUILD_MCNP2CAD_TESTS "Build mcnp2cad tests"                   ON)

# Default to a release build
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not specified, defaulting to Release")
  set(CMAKE_BUILD_TYPE Release)
endif ()
  
# Use C++11
set(CMAKE_CXX_STANDARD 11)

# iGeom assumed to have cone functionality
add_definitions(-DHAVE_IGEOM_CONE=ON)

# Required to find Eigen3 on Windows...
if(MSVC AND NOT STANDALONE_MCNP2CAD)
  set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/mcnp2cad/cmake ${CMAKE_MODULE_PATH})
endif()


# Find Eigen3
find_package(Eigen3 3.3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

# Find iGeom. If mcnp2cad is being built as a standalone library, then
# -DIGEOM_DIR must be specified. If it is not being built as a standalone
# library, then it is assumed that the location of the iGeom library is known
# and that the iGeom headers have already been included.
if (STANDALONE_MCNP2CAD)
  if (NOT IGEOM_DIR)
    message(FATAL_ERROR "-DIGEOM_DIR must be specified when building mcnp2cad"
                        " as a standalone library")
  else ()
    message(STATUS "Building standalone mcnp2cad")
  endif ()

  # Find location of iGeom headers
  find_path(IGEOM_INCLUDE_DIR NAMES iGeom.h HINTS ${IGEOM_DIR}/include)
  message(STATUS "IGEOM_INCLUDE_DIR: ${IGEOM_INCLUDE_DIR}")

  # Find location of iGeom library
  find_library(IGEOM_LIB NAMES iGeom HINTS ${IGEOM_DIR}/lib)
  get_filename_component(IGEOM_LIB_DIR ${IGEOM_LIB} DIRECTORY)
  message(STATUS "IGEOM_LIB_DIR: ${IGEOM_LIB_DIR}")
  message(STATUS "IGEOM_LIB: ${IGEOM_LIB}")

  # Create an imported target for iGeom
  add_library(iGeom SHARED IMPORTED)
  set_target_properties(iGeom PROPERTIES IMPORTED_LOCATION "${IGEOM_LIB}"
                        INTERFACE_INCLUDE_DIRECTORIES "${IGEOM_INCLUDE_DIR}"
                        INTERFACE_LINK_DIRECTORIES "${IGEOM_LIB_DIR}")
else ()
  message(STATUS "Building mcnp2cad as a component of another package."
                 " The iGeom target should already exist.")
endif ()

# RPATH handling
if (STANDALONE_MCNP2CAD)
  # Add all directories that have been linked via the link_directories() command
  # to the RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
  # Add the location of libraries installed by this project to the
  # INSTALL_RPATH_DIRS variable
  if (CMAKE_INSTALL_RPATH)
    set(INSTALL_RPATH_DIRS "${CMAKE_INSTALL_RPATH}:${CMAKE_INSTALL_PREFIX}/lib")
  else ()
    set(INSTALL_RPATH_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
  endif ()
endif ()


SET(MCNP2CAD_SRC_FILES geometry.cpp
                       GQ_Characterize.cpp
                       mcnp2cad.cpp
                       MCNPInput.cpp
                       volumes.cpp)

set(MCNP2CAD_PUB_HEADERS dataref.hpp
                         geometry.hpp
                         GQ_Characterize.hpp
                         mcnp2cad.hpp
                         MCNPInput.hpp
                         options.hpp
                         volumes.hpp
                         ${CMAKE_CURRENT_BINARY_DIR}/version.hpp)
        
configure_file(version.hpp.in version.hpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# mcnp2cad library
add_library(mcnp2cad SHARED ${MCNP2CAD_SRC_FILES})
target_link_libraries(mcnp2cad iGeom)
if (STANDALONE_MCNP2CAD)
  # Only install public headers if building standalone
  set_target_properties(mcnp2cad PROPERTIES PUBLIC_HEADER
                        "${MCNP2CAD_PUB_HEADERS}")
endif ()
install(TARGETS mcnp2cad LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)

# GQ tests
if (BUILD_MCNP2CAD_TESTS)
  add_executable(test_GQ test_GQ.cpp GQ_Characterize.cpp)
  add_test(NAME test_gq_characterization COMMAND test_GQ)
  enable_testing()
endif ()

# Build CLI if building standalone
if (STANDALONE_MCNP2CAD)
  message(STATUS "Building mcnp2cad CLI")
  add_subdirectory(cli)
endif ()
