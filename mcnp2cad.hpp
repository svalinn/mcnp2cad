#ifndef MCNP2CAD_HPP
#define MCNP2CAD_HPP

#include <fstream>

extern std::ofstream record;

bool convert_mcnp(std::string filename, bool plugin_build);
class GeometryContext;
std::string mcnp2cad_version( bool full );

#endif // MCNP2CAD_HPP
