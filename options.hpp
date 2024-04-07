#ifndef MCNP2CAD_OPTIONS_H
#define MCNP2CAD_OPTIONS_H

#include <string>
#include "options.hpp"

struct program_option_struct{
  bool verbose {false};
  bool debug {false};

  bool infinite_lattice_extra_effort {false};
  bool tag_materials {true};
  bool tag_importances {true};
  bool tag_cell_IDs {true};
  bool make_graveyard {true};
  bool imprint_geom {true};
  bool merge_geom {true};
  bool uwuw_names {true};
  bool din {false};
  bool dout {false};

#ifdef USE_CLI
  std::string output_file {OPT_DEFAULT_OUTPUT_FILENAME_CLI};
#else
  std::string output_file {OPT_DEFAULT_OUTPUT_FILENAME};
#endif

  std::string input_file {""};

  std::string igeom_init_options {""};

  bool override_tolerance {false};
  double specific_tolerance {0.0};
};

extern struct program_option_struct Gopt;

#define OPT_VERBOSE (Gopt.verbose || Gopt.debug)
#define OPT_DEBUG   (Gopt.debug)

#endif /* MCNP2CAD_OPTIONS_H */
