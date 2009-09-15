#ifndef MCNP2IGEOM_OPTIONS_H
#define MCNP2IGEOM_OPTIONS_H

struct program_option_struct{
  bool verbose;
  bool debug;

  bool tag_materials;
  bool make_graveyard;
  
  const char* output_file;
  const char* input_file;
  
  const char* igeom_init_options;

  bool override_tolerance;
  double specific_tolerance;
};

extern struct program_option_struct opt;

#define OPT_VERBOSE (opt.verbose || opt.debug)
#define OPT_DEBUG   (opt.debug)

#define OPT_DEFAULT_OUTPUT_FILENAME "out.sat"

#endif /* MCNP2IGEOM_OPTIONS_H */
