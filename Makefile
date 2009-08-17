#CGM_BASE_DIR = /home/cnerg/opt/CGMA
CGM_BASE_DIR = /local.hd/cnergg/sjackson/CGM-testing

#include ${CGM_BASE_DIR}/lib/cgm.make
include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc

LDFLAGS = -L/local.hd/cnergg/sjackson/CGM-testing/lib -L/home/cnerg/opt/cubit10.2/64/bin -liGeom -lcgm -lcubiti19 -Wl,-rpath=/home/cnerg/opt/cubit10.2/64/bin

CXXSOURCES = mcnp2igeom.cpp MCNPInput.cpp geometry.cpp
CXXOBJS = mcnp2igeom.o MCNPInput.o geometry.o

CXXFLAGS = -g -Wall -Wextra -Werror -DUSING_CUBIT 


mcnp2igeom: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} 
# ${IGEOM_LIBS}


geometry.o: geometry.cpp geometry.hpp dataref.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp
mcnp2igeom.o: mcnp2igeom.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2igeom *.o