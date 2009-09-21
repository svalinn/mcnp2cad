#CGM_BASE_DIR = /home/cnerg/opt/CGMA
CGM_BASE_DIR = /local.hd/cnergg/sjackson/CGM-testing3
CUBIT_BASE_DIR = /home/cnerg/opt/cubit10.2/64


#include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc

LDFLAGS = -L${CGM_BASE_DIR}/lib -L${CUBIT_BASE_DIR}/bin -liGeom -lcgm -lcubiti19 -Wl,-rpath=${CUBIT_BASE_DIR}/bin

IGEOM_CPPFLAGS = -I${CGM_BASE_DIR}/include 
CXXSOURCES = mcnp2igeom.cpp MCNPInput.cpp volumes.o geometry.cpp
CXXOBJS = mcnp2igeom.o MCNPInput.o volumes.o geometry.o

# Remove HAVE_IGEOM_CONE from the next line if using old iGeom implementation
CXXFLAGS = -O2 -g -Wall -Wextra -Werror -DUSING_CGMA -DHAVE_IGEOM_CONE

mcnp2igeom: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} 
# ${IGEOM_LIBS}


geometry.o: geometry.cpp geometry.hpp dataref.hpp
volumes.o: volumes.cpp volumes.hpp geometry.hpp MCNPInput.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp 
mcnp2igeom.o: mcnp2igeom.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp volumes.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2igeom *.o
