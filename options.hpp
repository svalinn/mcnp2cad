#ifndef MCNP2IGEOM_OPTIONS_H
#define MCNP2IGEOM_OPTIONS_H

#include <string>

struct program_option_struct{
  bool verbose;
  bool debug;

  bool tag_materials;
  bool tag_cell_IDs;
  bool make_graveyard;
  bool imprint_geom;
  bool merge_geom;
  
  std::string output_file;
  std::string input_file;
  
  std::string igeom_init_options;

  bool override_tolerance;
  double specific_tolerance;
};

extern struct program_option_struct opt;

#define OPT_VERBOSE (opt.verbose || opt.debug)
#define OPT_DEBUG   (opt.debug)

#define OPT_DEFAULT_OUTPUT_FILENAME "out.sat"

#endif /* MCNP2IGEOM_OPTIONS_H */
