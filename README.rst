THIS PROJECT IS NO LONGER SUPPORTED
======================================

This project relies on the iGeom interface to provide access to an
underlying CAD engine.  That interface is no longer actively supported
in any project and none of this historical implementations continue to
be valid due to licensing and other restrictions.

Since 2024, support for importing MCNP geometries into Cubit
has been incorporated natively into [Coreform Cubit](https://coreform.com/coreform-cubit/).
Users are directed to explore [Coreform Cubit](https://coreform.com/coreform-cubit/)
as a solution.  The verison of Cubit distributed by Sandia National Laboratories
does not yet include this support. Users of Sandia Cubit are encouraged to request
this capability through their Sandia points of contact.

In theory, it should be possible to revive this by providing access to
an open source CAD engine, such as OpenCascade (OCC), either by implementing
a lightweight version of iGeom for OCC or modifying this code to directly
access OCC.  The permissive license of this should allow others to tackle
this if/when they have interest.

Below is the original README file describing other aspects of this package.

MCNP2CAD README
===============

MCNP to iGeom/CAD converter: a program to extract the geomtery from MCNP
input files and write it out using any ITAPS iGeom backend.

This project creates a shared object file which is called by the Trelis plugin
or the command line interface created with the -DBUILD_CLI=true flag.

The following groups intend to support such implementations:

* CNERG_ will develop an iGeom-like interface as part of a Trelis_ & Cubit_
  plugin that uses this library to import MCNP geometry into Trelis/Cubit.

* The SIGMA_ team will develop an iGeom-like interface as part of an
  implementation of CGM_ that is based on the OpenCascade_ solid modeling
  engine.  This version may support a command line tool.

Bug reports are appreciated - please post an issue in our `Github repository
<https://github.com/svalinn/mcnp2cad/issues>`_.

This tool is based on an concept first developed at Argonne National
Laboratory.

Build Instructions:
--------------------

Using cmake:

Install Eigen3.

When installing mcnp2cad, be sure to include the following flags:

-DIGEOM_LIB_DIR= path to directory containing libigeom

-DIGEOM_INCLUDE_DIR= path to iGeom include directory

If building the command line interface, also include the following flag:

-DBUILD_CLI=yes

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

.. _CNERG: http://cnerg.engr.wisc.edu
.. _Trelis: http://csimsoft.com
.. _Cubit: http://cubit.sandia.gov
.. _SIGMA: http://sigma.mcs.anl.gov
.. _CGM: http://sigma.mcs.anl.gov/cgm-library/
.. _OpenCascade: https://www.opencascade.com/
