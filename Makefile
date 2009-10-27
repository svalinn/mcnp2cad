#CGM_BASE_DIR = /home/cnerg/opt/CGMA
CGM_BASE_DIR = /local.hd/cnergg/sjackson/CGM/acis


include ${CGM_BASE_DIR}/lib/iGeom-Defs.inc


CXXSOURCES = mcnp2cad.cpp MCNPInput.cpp volumes.cpp geometry.cpp
CXXOBJS = mcnp2cad.o MCNPInput.o volumes.o geometry.o

# Remove HAVE_IGEOM_CONE from the next line if using old iGeom implementation
CXXFLAGS = -g -Wall -Wextra -Werror -DUSING_CGMA -DHAVE_IGEOM_CONE


LDFLAGS = ${IGEOM_LIBS} -Wl,"${IGEOM_LTFLAGS}"

mcnp2cad: ${CXXOBJS} Makefile
	${CXX} ${CXXFLAGS} -o $@ ${CXXOBJS}  ${LDFLAGS} 


geometry.o: geometry.cpp geometry.hpp dataref.hpp
volumes.o: volumes.cpp volumes.hpp geometry.hpp MCNPInput.hpp
MCNPInput.o: MCNPInput.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp 
mcnp2cad.o: mcnp2cad.cpp MCNPInput.hpp geometry.hpp dataref.hpp options.hpp volumes.hpp

.cpp.o:
	${CXX} ${CXXFLAGS} ${IGEOM_CPPFLAGS} -o $@ -c $<

clean:
	rm -rf mcnp2cad *.o
