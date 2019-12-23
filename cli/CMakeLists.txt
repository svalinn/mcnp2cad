# Create the mcnp2cad commandline executable, convertMCNP.

SET(CLI_SRC_FILES mcnp2cadex.cpp
                  ProgOptions.cpp
                  ProgOptions.hpp)

add_definitions(-DUSE_CLI=ON)
#link_directories("/opt/Trelis-16.5/bin")
add_executable(convertMCNP ${CLI_SRC_FILES})
target_link_libraries(convertMCNP mcnp2cad)

# CLI inherits rpath functionality from mcnp2cad library
