
MCNP2CAD README
===============

MCNP to iGeom/CAD converter: a program to extract the geomtery from MCNP 
input files and write it out using any ITAPS iGeom backend. 

This project is currently in transition based on changes to the underlying CGM
library, with modified versions available as `a runtime plugin
<http://github.com/svalinn/DAGMC-Trelis>`_ to Trelis for an ACIS based
implementation and as a tool in `the CGM-ANL project
<http://sigma.mcs.anl.gov/cgm-library/>`_ for OpenCascade based
implementation.

Bug reports are appreciated.

This tool is based on an concept first developed at Argonne National
Laboratory.

Unsupported Features: 
-----------------------

* Could be added easily:
   * More direct control over the size of the sphere/graveyard in which
     the geometry sits
   * Support for interpolation (`nI/nILOG`) and multiplication (`nM`) syntax.
   * Make the title card optional, as it apparently is in MCNP.

* Could be added with effort:
   * Correct handling of hexagonal prism lattices for lattices based on irregular
     hexgons
   * Support for `ELL` (ellipse), `WED` (wedge), and `ARB` (arbitrary polyhedron) 
     macrobodies
   * Ability to refer to individual facets of macrobodies as surfaces in cell
     specification, using the `NNNN.MM` syntax.  Parsing support for this feature
     exists, but output support does not.
   * Support for lattices in universe 0
   * Faster/more efficient generation of embedded universes within lattices.
   * Complete support for `M=-1` argument in `TRn` (transform) cards.
   * Automatically annotate reflecting/white boundaries and periodic surfaces.
   * Support for 3-arg and 5-arg specification of rotations in transformations.
   * Support for jump (`nJ`) syntax: requires knowledge of default values in 
     many possible contexts

* Could be added with substantial effort:
   * General support for `SQ`, `GQ`, `X`, `Y`, and `Z` surfaces. (Elliptic Cylinders and Elliptic Cones currently supported.)
     (Simple `X`, `Y`, `Z` surfaces are already supported.)
   * Robust error detection and reporting for ill-formed MCNP inputs.


