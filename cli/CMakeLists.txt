# Create the mcnp2cad commandline executable, convertMCNP.

SET(CLI_SRC
    mcnp2cadex.cpp
    ProgOptions.cpp
    ProgOptions.hpp)

add_definitions(-DUSE_CLI)
add_executable(convertMCNP ${CLI_SRC})
target_link_libraries(convertMCNP mcnp2cad)
if(IGEOM_LIB_DIR)
    set_target_properties(convertMCNP PROPERTIES INSTALL_RPATH ${IGEOM_LIB_DIR} BUILD_WITH_INSTALL_RPATH TRUE)
elseif(CMAKE_PREFIX_PATH)
    set_target_properties(convertMCNP PROPERTIES INSTALL_RPATH ${CMAKE_PREFIX_PATH} BUILD_WITH_INSTALL_RPATH TRUE)
endif()
