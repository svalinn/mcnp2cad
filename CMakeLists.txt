project(libMCNP2CAD)

cmake_minimum_required(VERSION 2.8)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DHAVE_IGEOM_CONE" )

# adjust compiler setting for Linux using gcc version 5.0 and higher
if(CMAKE_SYSTEM_NAME MATCHES Linux AND CMAKE_COMPILER_IS_GNUCXX)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "5.0")
    if(NOT CMAKE_CXX_FLAGS MATCHES _GLIBCXX_USE_CXX11_ABI)
      set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0 ${CMAKE_CXX_FLAGS}")
    endif()
  endif()
endif()

if(${BUILD_CLI})
    add_subdirectory(${CMAKE_SOURCE_DIR}/cli)
endif()

# These directories need to be defined by the user.
include_directories(${IGEOM_INCLUDE_DIR})

# only require iGeom if this project is being built as a standalone
if(${PROJECT_SOURCE_DIR} EQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  find_library(IGEOM_LIB NAMES iGeom HINTS ${IGEOM_LIB_DIR})
else()
  # if this is part of another project
  # iGeom sould be set as a native CMake dependency
  set(IGEOM_LIB iGeom)
endif()

SET(LIB_SRC
    mcnp2cad.cpp
    mcnp2cad.hpp
    MCNPInput.cpp
    MCNPInput.hpp
    geometry.cpp
    geometry.hpp
    volumes.cpp
    volumes.hpp)

add_library(mcnp2cad SHARED ${LIB_SRC})
target_link_libraries(mcnp2cad ${IGEOM_LIB} armadillo)
