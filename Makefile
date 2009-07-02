CGM_BASE_DIR = /home/cnerg/opt/CGMA

include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc

LDFLAGS = ${IGEOM_LDFLAGS}

CXXSOURCES = mcnp2igeom.cpp
CXXOBJS = mcnp2igeom.o

CXXFLAGS = -g -Wall -Werror

mcnp2igeom: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} ${IGEOM_LIBS}

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2igeom *.o