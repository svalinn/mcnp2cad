#ifndef MCNP2CAD_VOLUMES_H
#define MCNP2CAD_VOLUMES_H

#include <cstdlib>
#include "iGeom.h"


class Transform;
class SurfaceCard;

class SurfaceVolume{

protected:
  const Transform* transform;

public:
  SurfaceVolume( const Transform* transform_p = NULL):
    transform(transform_p)
  {}
  virtual ~SurfaceVolume(){}

  void setTransform( const Transform* transform_p ){ transform = transform_p; }
  
  virtual double getFarthestExtentFromOrigin( ) const = 0;
  virtual iBase_EntityHandle define( bool positive, iGeom_Instance& igm, double world_size );

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ) = 0;
};

class VolumeCache;

/** 
 * Function to create an SurfaceVolume object from a SurfaceCard.
 * Created volumes are kept in a cache.  If the v parameter is null,
 * a default cache (static within volumes.cpp) will be used.  If multiple
 * MCNPInput instances are used in the course of a single program, then
 * there will need to be a way to get multiple VolumeCache objects;
 * that's not an issue right now, so I haven't coded it.
 */

extern 
SurfaceVolume& makeSurface( const SurfaceCard* card, VolumeCache* v = NULL, int facet = 0 );

//extern
SurfaceVolume* FacetSurface( const std::string mnemonic, const std::vector< double > args, int facet );

extern
SurfaceVolume* rccFacet( const std::vector< double > args, int facet );

extern
SurfaceVolume* boxFacet( const std::vector< double > args, int facet );

extern
SurfaceVolume* rppFacet( const std::vector< double > args, int facet );

extern
SurfaceVolume* hexFacet( const std::vector< double > args, int facet );

extern
SurfaceVolume* recFacet( const std::vector< double > args, int facet );

extern 
iBase_EntityHandle makeWorldSphere( iGeom_Instance& igm, double world_size ); 

extern
iBase_EntityHandle applyTransform( const Transform& t, iGeom_Instance& igm, iBase_EntityHandle& e );

extern
iBase_EntityHandle applyReverseTransform( const Transform& tx, iGeom_Instance& igm, iBase_EntityHandle& e ) ;






// TODO: clean this igeom check function up
#define CHECK_BUF_SIZE 512
static char m_buf[CHECK_BUF_SIZE];

#define CHECK_IGEOM(err, msg) \
  do{/*std::cout << msg << std::endl;*/ if((err) != iBase_SUCCESS){     \
    std::cerr << "iGeom error (" << err << "): " << msg << std::endl;   \
    iGeom_getDescription( igm, m_buf, CHECK_BUF_SIZE);                  \
    std::cerr << " * " << m_buf << std::endl;                           \
     } } while(0) 

#endif /* MCNP2CAD_VOLUMES_H */
