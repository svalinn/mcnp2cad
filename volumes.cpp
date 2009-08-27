#include <iostream>
#include <map>

#include <cassert>

#include "MCNPInput.hpp"
#include "volumes.hpp"
#include "geometry.hpp"


static Vector3d origin(0,0,0);


iBase_EntityHandle makeWorldSphere( iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle world_sphere;
  int igm_result;
  iGeom_createSphere( igm, world_size, &world_sphere, &igm_result);
  CHECK_IGEOM( igm_result, "making world sphere" );
  return world_sphere;
}



class PlaneSurface : public SurfaceVolume { 

public:
  Vector3d normal;
  double offset;

public:
  PlaneSurface( const Vector3d& normal_p, double offset_p ) :
    SurfaceVolume(), normal(normal_p), offset(offset_p) 
  {}
  
  virtual double getFarthestExtentFromOrigin() const{
    // this is a funny situation, since planes are technically infinte...
    // in order to have a sane answer, we just return the offset from the origin.
    // (multiplied by root 3, which was done in the old converter, why?)
    return sqrt(3.0) *  std::abs(offset);
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size){

    int igm_result;
    iBase_EntityHandle world_sphere = makeWorldSphere(igm, world_size);
    iBase_EntityHandle hemisphere;
    // note the reversal of sense in this call; mcnp and igeom define it differently.
    iGeom_sectionEnt( igm, world_sphere, 
		      normal.v[0], normal.v[1], normal.v[2], offset, !positive, &hemisphere, &igm_result);
    CHECK_IGEOM( igm_result, "Sectioning world for a plane" );
    return hemisphere;


  }

};

class CylinderSurface : public SurfaceVolume {

public:
  enum axis{ X=0, Y=1, Z=2 } axis;
  double radius;
  Vector3d center;
  bool onaxis;

  CylinderSurface( enum axis axis_p, double radius_p ):
    SurfaceVolume(), axis(axis_p), radius(radius_p), center(origin), onaxis(true)
  {}

  CylinderSurface( enum axis axis_p, double radius_p, double trans1, double trans2 ):
    SurfaceVolume(), axis(axis_p), radius(radius_p), center(origin), onaxis(false)
  {
    switch(axis){
    case X: center.v[Y] += trans1; center.v[Z] += trans2; break;
    case Y: center.v[X] += trans1; center.v[Z] += trans2; break;
    case Z: center.v[X] += trans1; center.v[Y] += trans2; break;
    }
  }
  
  virtual double getFarthestExtentFromOrigin( ) const{
    return radius + center.length();
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;

    iBase_EntityHandle cylinder;
    iGeom_createCylinder( igm, 2.0 * world_size, radius, 0, &cylinder, &igm_result);
    CHECK_IGEOM( igm_result, "making cylinder" );

    
    if( axis == X ){
      iGeom_rotateEnt( igm, cylinder, 90, 0, 1, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (X)" );
    }
    else if( axis == Y ){
      iGeom_rotateEnt( igm, cylinder, 90, 1, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (Y)" );
    }

    if( onaxis == false ){
      iGeom_moveEnt( igm, cylinder, center.v[0], center.v[1], center.v[2], &igm_result);
      CHECK_IGEOM( igm_result, "moving cylinder" );
    }

    iBase_EntityHandle world_sphere = makeWorldSphere( igm, world_size );
    iBase_EntityHandle final_cylinder;

    if( positive ){
      iGeom_subtractEnts( igm, world_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making positive cylinder" );
    }
    else{
      iGeom_intersectEnts( igm, world_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making negative cylinder" );
    }

    return final_cylinder;
  };

};

#ifdef HAVE_IGEOM_CONE

class ConeSurface : public SurfaceVolume {

public:

  enum nappe {LEFT=-1, BOTH=0, RIGHT=1};

  static enum nappe make_nappe( double param ){
    
    enum nappe n = static_cast<enum nappe>(param);
    if( -1 <= n && n <= 1 ){
      return n;
    }
    else{
      std::cerr << "WARNING: Bad cylinder +/-1 argument: " << param << std::endl;
      std::cerr << "         will pretend it was really 0" << std::endl;
      return BOTH;
    }
  }

  enum axis{ X=0, Y=1, Z=2 } axis;
  double theta;    /// the cone's opening angle
  Vector3d center; /// the cone's apex 
  bool onaxis;
  enum nappe nappe; 

  ConeSurface( enum axis axis_p, double tsquared_p, double point_p, double nappe_p ):
    SurfaceVolume(), axis(axis_p), theta( atan(sqrt(tsquared_p)) ), center(origin), onaxis(true), nappe(make_nappe(nappe_p))
  {
    center.v[axis] = point_p; 
  }

  ConeSurface( enum axis axis_p, double tsquared_p, Vector3d center_p, double nappe_p ):
    SurfaceVolume(), axis(axis_p), theta( atan(sqrt(tsquared_p)) ), center(center_p), onaxis(false), nappe(make_nappe(nappe_p))
  {}
  
  virtual double getFarthestExtentFromOrigin( ) const{
    return sqrt(3) + ( center.length() );
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    double height = (center.length() + world_size);
    double base_radius = height * tan( theta / 2 );

    int igm_result;

    iBase_EntityHandle right_nappe = 0;
    iBase_EntityHandle left_nappe = 0;
    iBase_EntityHandle cone; 
    
    if( nappe != LEFT){
      iGeom_createCone( igm, height, base_radius, 0, 0, &right_nappe, &igm_result);
      CHECK_IGEOM( igm_result, "making cone (right nappe)" );
      iGeom_rotateEnt( igm, right_nappe, 180, 1, 0, 0, &igm_result);
      CHECK_IGEOM( igm_result, "Rotating cone (right nappe)");
      iGeom_moveEnt( igm, right_nappe, 0, 0, height/2.0, &igm_result );
      CHECK_IGEOM( igm_result, "Moving cone (right nappe)");      
      cone = right_nappe;
    }
    if( nappe != RIGHT ){
      iGeom_createCone( igm, height, base_radius, 0, 0, &left_nappe, &igm_result );
      CHECK_IGEOM( igm_result, "making cone (left nappe)" );
      iGeom_moveEnt( igm, left_nappe, 0, 0, -height/2.0, &igm_result );
      CHECK_IGEOM( igm_result, "Moving cone (left nappe)" );
      cone = left_nappe;
    }

    
    if( right_nappe && left_nappe ){
      iBase_EntityHandle nappes[2] = {right_nappe, left_nappe};
      iGeom_uniteEnts( igm, nappes, 2, &cone, &igm_result );
      CHECK_IGEOM( igm_result, "Unioning cone nappes" );
    }

    if( axis == X ){
      iGeom_rotateEnt( igm, cone, 90, 0, 1, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cone (X)" );
    }
    else if( axis == Y ){
      iGeom_rotateEnt( igm, cone, -90, 1, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cone (Y)" );
    }

    iGeom_moveEnt( igm, cone, center.v[0], center.v[1], center.v[2], &igm_result);
    CHECK_IGEOM( igm_result, "moving cone to its apex" );

    iBase_EntityHandle world_sphere = makeWorldSphere( igm, world_size );
    iBase_EntityHandle final_cone;

    if( positive ){
      iGeom_subtractEnts( igm, world_sphere, cone, &final_cone, &igm_result);
      CHECK_IGEOM( igm_result, "making positive cone" );
    }
    else{
      iGeom_intersectEnts( igm, world_sphere, cone, &final_cone, &igm_result);
      CHECK_IGEOM( igm_result, "making negative cone" );
    }
    return final_cone;

    }

};

#endif /* HAVE_IGEOM_CONE */

class SphereSurface : public SurfaceVolume {

public:
  Vector3d center;
  double radius;

public:
  SphereSurface( const Vector3d& center_p, double radius_p ) :
    SurfaceVolume(), center(center_p), radius(radius_p)
  {}

  virtual ~SphereSurface(){}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return (center.length() + radius);
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    int igm_result;
    iBase_EntityHandle sphere;

    iGeom_createSphere( igm, radius, &sphere, &igm_result);
    CHECK_IGEOM( igm_result, "making sphere" );

    iGeom_moveEnt( igm, sphere, center.v[0], center.v[1], center.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "moving sphere" );

    // sphere now defines the interior sphere volume, corresponding to mcnp's notion of negative sense.
    // If positive sense is required, we must return the outside of the surface.

    if(positive){

	iBase_EntityHandle world_sphere = makeWorldSphere(igm, world_size);
	iBase_EntityHandle volume;

	iGeom_subtractEnts( igm, world_sphere, sphere, &volume, &igm_result);
	CHECK_IGEOM( igm_result, "subtracting sphere" );
	
	sphere = volume;

    }
    
    return sphere; 
  }

};





class VolumeCache{

protected:
  std::map<const SurfaceCard*,SurfaceVolume*> mapping;

public:
  VolumeCache(){}
  
  bool contains( const SurfaceCard* c ) const {
    return ( mapping.find(c) != mapping.end() );
  }

  SurfaceVolume* get( const SurfaceCard* c ) {
    assert( contains( c ) );
    return (*(mapping.find(c))).second;
  }

  void insert( const SurfaceCard* c, SurfaceVolume* s ){
    mapping[c] = s;
  }

};

// the VolumeCache to use if none is provided to makeSurface() calls.
static VolumeCache default_volume_cache;

SurfaceVolume& makeSurface( const SurfaceCard* card, VolumeCache* v){
  
  VolumeCache& cache = default_volume_cache;
  if( v != NULL ){
    cache = *v;
  }

  
  if( cache.contains( card ) ){
    return *cache.get( card );
  }
  else{ 
    // SurfaceCard variables:  mnemonic, args, coord_xform  
    SurfaceVolume* surface;
    const std::string& mnemonic = card->getMnemonic();
    const std::vector< double >& args = card->getArgs(); 
    
    if( mnemonic == "so"){
      surface = new SphereSurface( origin, args.at(0) );
    }
    else if( mnemonic == "sx"){
      surface = new SphereSurface( Vector3d( args.at(0), 0, 0 ), args.at(1) );
    }
    else if( mnemonic == "sy"){
      surface = new SphereSurface( Vector3d( 0, args.at(0), 0 ), args.at(1) );
    }
    else if( mnemonic == "sz"){
      surface = new SphereSurface( Vector3d( 0, 0, args.at(0) ), args.at(1) );
    }
    else if( mnemonic == "s" || mnemonic == "sph" ){
      surface = new SphereSurface( Vector3d( args ), args.at(3) );
    }
    else if( mnemonic == "p"){
      if( args.size() > 4 ){
	std::cerr << "Warning: surface of type P with more than 4 parameters." << std::endl;
	std::cerr << "         This surface type is unsupported; will proceed as if only 4 parameters given,\n";
	std::cerr << "         but the result will probably be incorrect!" << std::endl;
      }
      Vector3d v(args);
      surface = new PlaneSurface( v, args.at(3)/v.length() );
    }
    else if( mnemonic == "px"){
      surface = new PlaneSurface( Vector3d( 1, 0, 0), args.at(0) );
    }
    else if( mnemonic == "py"){
      surface = new PlaneSurface( Vector3d( 0, 1, 0), args.at(0) );
    }
    else if( mnemonic == "pz"){
      surface = new PlaneSurface( Vector3d( 0, 0, 1), args.at(0) );
    }
    else if( mnemonic == "cx" ){
      surface = new CylinderSurface( CylinderSurface::X, args.at(0) );
    }
    else if( mnemonic == "cy" ){
      surface = new CylinderSurface( CylinderSurface::Y, args.at(0) );
    }
    else if( mnemonic == "cz" ){
      surface = new CylinderSurface( CylinderSurface::Z, args.at(0) );
    }
    else if( mnemonic == "c/x"){
      surface = new CylinderSurface( CylinderSurface::X, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/y"){
      surface = new CylinderSurface( CylinderSurface::Y, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/z"){
      surface = new CylinderSurface( CylinderSurface::Z, args.at(2), args.at(0), args.at(1) );
    }
#ifdef HAVE_IGEOM_CONE
    else if( mnemonic == "kx"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( ConeSurface::X, args.at(1), args.at(0), arg3 );
    } 
    else if( mnemonic == "ky"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( ConeSurface::Y, args.at(1), args.at(0), arg3 );
    }
    else if( mnemonic == "kz"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( ConeSurface::Z, args.at(1), args.at(0), arg3 );
    }
    else if( mnemonic == "k/x" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( ConeSurface::X, args.at(3), Vector3d(args),  arg5 );
    }
    else if( mnemonic == "k/y" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( ConeSurface::Y, args.at(3), Vector3d(args),  arg5 );
    }
    else if( mnemonic == "k/z" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( ConeSurface::Z, args.at(3), Vector3d(args),  arg5 );
    }
#endif /*HAVE_IGEOM_CONE */
    else{
      throw std::runtime_error( mnemonic + " is not a supported surface" );
    }
    
    if( card->getTransform().hasData() ){
      const Transform& transform = card->getTransform().getData();
      surface->setTransform( &transform );
    }

    cache.insert( card, surface );
    return *surface;

  }


}


