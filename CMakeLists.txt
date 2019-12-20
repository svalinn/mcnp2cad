project(libMCNP2CAD)

cmake_minimum_required(VERSION 2.8)

# Build options
option(BUILD_MCNP2CAD_TESTS "Build mcnp2cad tests" ON)

# Default to a release build
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not specified, defaulting to Release")
  set(CMAKE_BUILD_TYPE Release)
endif ()
  
# Use C++11
set(CMAKE_CXX_STANDARD 11)

add_definitions(-DHAVE_IGEOM_CONE=ON)

# Adjust compiler setting for Linux using gcc version 5.0 and higher
if (CMAKE_SYSTEM_NAME MATCHES Linux AND CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0")
    if (NOT CMAKE_CXX_FLAGS MATCHES _GLIBCXX_USE_CXX11_ABI)
      set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0 ${CMAKE_CXX_FLAGS}")
    endif ()
  endif ()
endif ()

if (BUILD_CLI)
  add_subdirectory(cli)
endif ()

# IGEOM_DIR must be specified if mcnp2cad is being built as a standalone
# library. If it is not specified, then CMake will assume that mcnp2cad is being
# built as a part of a larger project which already knows the location of iGeom.
if (IGEOM_DIR)
  message(STATUS "Building standalone mcnp2cad")
  set(STANDALONE ON)
  include_directories(${IGEOM_DIR}/include)
  find_library(IGEOM_LIB NAMES iGeom HINTS ${IGEOM_DIR}/lib)
  get_filename_component(IGEOM_LIB_DIR ${IGEOM_LIB} DIRECTORY)
  link_directories(${IGEOM_LIB_DIR})
  message(STATUS "IGEOM_LIB_DIR: ${IGEOM_LIB_DIR}")
else ()
  set(STANDALONE OFF)
endif ()

# RPATH handling. If CMAKE_INSTALL_RPATH_USE_LINK_PATH is set to ON, then all
# directories that have been linked via the link_directories() command will be
# added to the RPATH.
if (STANDALONE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
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
                         version.hpp
                         volumes.hpp)

# mcnp2cad library
add_library(mcnp2cad SHARED ${MCNP2CAD_SRC_FILES})
set_target_properties(mcnp2cad PROPERTIES PUBLIC_HEADER
                      "${MCNP2CAD_PUB_HEADERS}")
target_link_libraries(mcnp2cad iGeom)
if (STANDALONE)
  set(INSTALL_INCLUDE_DIR include)
else ()
  set(INSTALL_INCLUDE_DIR include/mcnp2cad)
endif ()
install(TARGETS mcnp2cad LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION ${INSTALL_INCLUDE_DIR})

# GQ tests
if (BUILD_MCNP2CAD_TESTS)
  add_executable(test_GQ test_GQ.cpp GQ_Characterize.cpp)
  target_link_libraries(test_GQ)
  add_test(NAME test_gq_characterization COMMAND test_GQ)
  enable_testing()
endif ()
