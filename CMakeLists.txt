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
find_library(IGEOM_LIB NAMES iGeom HINTS ${IGEOM_LIB_DIR})

SET(LIB_SRC
    mcnp2cad.cpp
    mcnp2cad.hpp
    MCNPInput.cpp
    MCNPInput.hpp
    geometry.cpp
    geometry.hpp
    volumes.cpp
    volumes.hpp
    GQ_Characterize.cpp
    GQ_Characterize.hpp
    test_GQ.cpp
)

add_library(mcnp2cad SHARED ${LIB_SRC})
target_link_libraries(mcnp2cad ${IGEOM_LIB})

add_executable(test_GQ test_GQ.cpp GQ_Characterize.cpp)
target_link_libraries(test_GQ)
if(IGEOM_LIB_DIR)
    set_target_properties(test_GQ PROPERTIES INSTALL_RPATH ${IGEOM_LIB_DIR} BUILD_WITH_INSTALL_RPATH TRUE)
elseif(CMAKE_PREFIX_PATH)
    set_target_properties(test_GQ PROPERTIES INSTALL_RPATH ${CMAKE_PREFIX_PATH} BUILD_WITH_INSTALL_RPATH TRUE)
endif()

enable_testing()
add_test(NAME test_gqs COMMAND test_GQ)
