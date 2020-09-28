project(libMCNP2CAD)

cmake_minimum_required(VERSION 2.8)

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

# Adjust compiler setting for Linux using gcc version 5.0 and higher
if (CMAKE_SYSTEM_NAME MATCHES Linux AND CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0")
    if (NOT CMAKE_CXX_FLAGS MATCHES _GLIBCXX_USE_CXX11_ABI)
      set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0 ${CMAKE_CXX_FLAGS}")
    endif ()
  endif ()
endif ()

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
  include_directories(${IGEOM_INCLUDE_DIR})
  message(STATUS "IGEOM_INCLUDE_DIR: ${IGEOM_INCLUDE_DIR}")

  # Find location of iGeom library and create an imported target for it
  find_library(IGEOM_LIB NAMES iGeom HINTS ${IGEOM_DIR}/lib)
  get_filename_component(IGEOM_LIB_DIR ${IGEOM_LIB} DIRECTORY)
  link_directories(${IGEOM_LIB_DIR})
  add_library(iGeom SHARED IMPORTED)
  set_target_properties(iGeom PROPERTIES IMPORTED_LOCATION "${IGEOM_LIB}")
  message(STATUS "IGEOM_LIB_DIR: ${IGEOM_LIB_DIR}")
  message(STATUS "IGEOM_LIB: ${IGEOM_LIB}")
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
                         version.hpp
                         volumes.hpp)

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
  target_link_libraries(test_GQ)
  add_test(NAME test_gq_characterization COMMAND test_GQ)
  enable_testing()
endif ()

# Build CLI if building standalone
if (STANDALONE_MCNP2CAD)
  message(STATUS "Building mcnp2cad CLI")
  add_subdirectory(cli)
endif ()
