#include "options.hpp"
#include "mcnp2cad.hpp"
#include "ProgOptions.hpp"

//Stores all of the options selected
struct program_option_struct Gopt;

int main(int argc, char* argv[]){

  // set default options
  Gopt.verbose = Gopt.debug = false;
  Gopt.infinite_lattice_extra_effort = false;
  Gopt.tag_materials = true;
  Gopt.tag_importances = true;
  Gopt.tag_cell_IDs = true;
  Gopt.make_graveyard = true;
  Gopt.imprint_geom = true;
  Gopt.merge_geom = true;
  Gopt.input_file = "";
  Gopt.output_file = OPT_DEFAULT_OUTPUT_FILENAME;
  Gopt.igeom_init_options = "";
  Gopt.override_tolerance = false;
  Gopt.uwuw_names = false;
  Gopt.din = false;
  Gopt.dout = false;

  ProgOptions po("mcnp2cad " + mcnp2cad_version(false) +  ": An MCNP geometry to CAD file converter");
  po.setVersion( mcnp2cad_version(true) );

  po.addOpt<void>("extra-effort,e","Use extra effort to get infinite lattices right (may be slow)", 
                  &Gopt.infinite_lattice_extra_effort );
  po.addOpt<void>("verbose,v", "Verbose output", &Gopt.verbose );
  po.addOpt<void>("debug,D", "Debugging (very verbose) output", &Gopt.debug );
  po.addOpt<void>("Di", "Debug output for MCNP parsing phase only", &Gopt.din);
  po.addOpt<void>("Do","Debug output for iGeom output phase only", &Gopt.dout);

  po.addOptionHelpHeading( "Options controlling CAD output:" );
  po.addOpt<std::string>(",o", "Give name of output file. Default: " + Gopt.output_file, &Gopt.output_file );
  po.addOpt<double>("tol,t", "Specify a tolerance for merging surfaces", &Gopt.specific_tolerance );
  po.addOpt<void>("skip-mats,M", "Do not tag materials using group names", 
                  &Gopt.tag_materials, po.store_false );
  po.addOpt<void>("skip-imps,P", "Do not tag cell importances using group names",
                  &Gopt.tag_importances, po.store_false );
  po.addOpt<void>("skip-nums,N", "Do not tag cell numbers using body names",
                  &Gopt.tag_cell_IDs, po.store_false );
  po.addOpt<void>("skip-merge,E", "Do not merge the geometry",
                  &Gopt.merge_geom, po.store_false );
  po.addOpt<void>("skip-imprint,I", "Do not imprint the geometry; implies skip-merge",
                  &Gopt.imprint_geom, po.store_false );
  po.addOpt<void>("skip-graveyard,G", "Do not bound the geometry with a `graveyard' bounding box",
                  &Gopt.make_graveyard, po.store_false );
  po.addOpt<void>("uwuw-names,U", "Use a UWUW compatible name scheme for material groups,"
                                   "i.e. 'mat:mX/rho:Y' where X is material number is Y is density",
                  &Gopt.uwuw_names, po.store_true );

  po.addRequiredArg( "input_file", "Path to MCNP geometry input file", &Gopt.input_file );

  po.parseCommandLine( argc, argv );

  if( po.numOptSet( "tol,t" ) ){
    Gopt.override_tolerance = true;
    if( Gopt.specific_tolerance <= 0.0 || Gopt.specific_tolerance > .1 ){
      std::cerr << "Warning: you seem to have specified an unusual tolerance (" 
                << Gopt.specific_tolerance << ")." << std::endl;
    }
  }


  if( Gopt.merge_geom && !Gopt.imprint_geom ) {
    std::cerr << "Warning: cannot merge geometry without imprinting, will skip merge too." << std::endl;
  }

  convert_mcnp( Gopt.input_file, false );
  convert_mcnp( "file", false );

}
