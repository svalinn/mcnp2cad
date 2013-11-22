# Makefile for mcnp2cad
#
# Needs to be called while setting the CGM_BASE_DIR variable.
# CGM_BASE_DIR should be the directory in which there is an
# 'include' and 'lib' directory with the CGM include files and
# libraries.
#
# prompt%> make CGM_BASE_DIR=/path/to/installed/cgm
#
# CGM_BASE_DIR = /path/to/installed/cgm

include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc


CXXSOURCES = mcnp2cad.cpp MCNPInput.cpp volumes.cpp geometry.cpp ProgOptions.cpp
CXXOBJS = mcnp2cad.o MCNPInput.o volumes.o geometry.o ProgOptions.o

# Remove HAVE_IGEOM_CONE from the next line if using old iGeom implementation
CXXFLAGS = -g -Wall -Wextra -DUSING_CGMA -DHAVE_IGEOM_CONE


LDFLAGS = ${IGEOM_LIBS} 

mcnp2cad: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS} ${LDFLAGS}
# The following may be more convenient than the above on Linux
#	libtool --mode=link ${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} 


geometry.o: geometry.cpp geometry.hpp dataref.hpp
volumes.o: volumes.cpp volumes.hpp geometry.hpp MCNPInput.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp 
mcnp2cad.o: mcnp2cad.cpp MCNPInput.hpp geometry.hpp dataref.hpp \
            options.hpp volumes.hpp ProgOptions.hpp version.hpp
ProgOptions.o: ProgOptions.cpp ProgOptions.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2cad *.o
