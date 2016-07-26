mcnp2cad
=========

MCNP to iGeom/CAD converter: a program to extract the geomtery from MCNP 
input files and write it out using any ITAPS iGeom backend. 

This program is currently maintained by Julie Zachman (zachman@engr.wisc.edu).
Bug reports are appreciated.

The home of mcnp2cad on the web is https://github.com/svalinn/mcnp2cad

This tool is based on an concept first developed at Argonne National
Laboratory.

Compiling (with CGM):
---------------------

At present CGM_BASE_DIR must be specifed as a make paramter, e.g.
::
   make CGM_BASE_DIR=<path to CGM>

CGM_BASE_DIR must point to a valid installation of the CGM library.  Information and instructions
for getting and using CGM are available at https://trac.mcs.anl.gov/projects/ITAPS/wiki/CGM

In addition, any shared libraries required by CGM must be located either in 
standard system directories, or in a location pointed to by the 
LD_LIBRARY_PATH environment variable.  If your version of CGM was compiled 
against CUBIT, you  will likely need to set LD_LIBRARY_PATH as follows:
::
    export LD_LIBRARY_PATH=/path/to/cubit13.1/bin 

Running:
---------

The program requires a single command-line argument: the path to an MCNP
input file.  Optionally, the -o flag can be used to specify the file to
which output should be written.  The iGeom backend will choose the output
format based on the extension of the output file name; .sat is the default
format for an ACIS-based or CUBIT-based CGM.

Help on other command-line flags is available by invoking the program with
the ``-h`` (or ``--help``) flag.  

The program will, by default, create a boundary volume named "graveyard"
around all created geometry.  This volume is needed for DAGMC analysis,
but users who are only interested in visualization may want to use the ``-G``
flag to turn the graveyard volume off. 

Unsupported Features: 
-----------------------

* Could be added easily:
   * More direct control over the size of the sphere/graveyard in which
     the geometry sits
   * Support for interpolation (``nI/nILOG``) and multiplication (``nM``) syntax.
   * Make the title card optional, as it apparently is in MCNP.

* Could be added with effort:
   * Correct handling of hexagonal prism lattices for lattices based on irregular
     hexgons
   * Support for ``ELL`` (ellipse), ``WED`` (wedge), and ``ARB`` (arbitrary polyhedron) 
     macrobodies
   * Support for lattices in universe 0
   * Faster/more efficient generation of embedded universes within lattices.
   * Complete support for ``M=-1`` argument in ``TRn`` (transform) cards.
   * Automatically annotate reflecting/white boundaries and periodic surfaces.
   * Support for 3-arg and 5-arg specification of rotations in transformations.
   * Support for jump (``nJ``) syntax: requires knowledge of default values in 
     many possible contexts

* Could be added with substantial effort:
   * General support for ``SQ``, ``GQ``, ``X``, ``Y``, and ``Z`` surfaces.  
     (Simple ``X``, ``Y``, ``Z`` surfaces are already supported.)
   * Robust error detection and reporting for ill-formed MCNP inputs.


Compiling a 32-bit mcnp2cad: 
-----------------------------

Users on 64-bit Linux may need to compile a 32-bit version of this program 
under certain conditions, for example, if only a 32-bit copy of Cubit 10.2 
is available.  This is a two-step process:

* Ensure that CGM is built for 32-bit by configuring it with the ``--enable-32bit`` 
  flag, and
* Add ``-m32`` to the ``CXXFLAGS`` in the mcnp2cad Makefile.

