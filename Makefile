CGM_BASE_DIR = /home/cnerg/opt/CGMA

include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc

LDFLAGS = ${IGEOM_LDFLAGS}

CXXSOURCES = mcnp2igeom.cpp MCNPInput.cpp geometry.cpp
CXXOBJS = mcnp2igeom.o MCNPInput.o geometry.o

CXXFLAGS = -g -Wall -Wextra -Werror -DUSING_CUBIT


mcnp2igeom: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} ${IGEOM_LIBS}


geometry.o: geometry.cpp geometry.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp
mcnp2igeom.o: mcnp2igeom.cpp MCNPInput.hpp geometry.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2igeom *.o