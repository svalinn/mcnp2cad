# Create the mcnp2cad commandline executable, convertMCNP.

SET(CLI_SRC_FILES mcnp2cadex.cpp
                  ProgOptions.cpp
                  ProgOptions.hpp)

add_definitions(-DUSE_CLI=ON)
add_executable(convertMCNP ${CLI_SRC_FILES})
target_link_libraries(convertMCNP mcnp2cad)
set_target_properties(convertMCNP PROPERTIES
                      INSTALL_RPATH "${INSTALL_RPATH_DIRS}")
install(TARGETS convertMCNP DESTINATION bin)
