#include <iostream>
#include <map>

#include <cassert>

#include "MCNPInput.hpp"
#include "volumes.hpp"
#include "geometry.hpp"
#include "options.hpp"


static Vector3d origin(0,0,0);


iBase_EntityHandle makeWorldSphere( iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle world_sphere;
  int igm_result;
  iGeom_createSphere( igm, world_size, &world_sphere, &igm_result);
  CHECK_IGEOM( igm_result, "making world sphere" );
  return world_sphere;
}

/**
 * A convenience function for SurfaceVolumes to call at the end of getHandle functions.
 * Return the negative or positive sense of the given body, as appropriate.  If
 * bound_within_world is true, a negative-sense body will be intersected with the world sphere
 * (a step necessary for cylinders and other infinite bodies) 
 */
static iBase_EntityHandle embedWithinWorld( bool positive, iGeom_Instance& igm, double world_size, 
					    iBase_EntityHandle body, bool bound_with_world )
{
  iBase_EntityHandle final_body;
  int igm_result;

  if( !positive && !bound_with_world ){
    final_body = body;
  }
  else{
    iBase_EntityHandle world_sphere = makeWorldSphere( igm, world_size );    
    
    if( positive ){
      iGeom_subtractEnts( igm, world_sphere, body, &final_body, &igm_result);
      CHECK_IGEOM( igm_result, "making positive body" );
    }
    else{ // !positive && bound_with_world
      iGeom_intersectEnts( igm, world_sphere, body, &final_body, &igm_result);
      CHECK_IGEOM( igm_result, "making negative body" );
    }
  }
  return final_body;
}


class PlaneSurface : public SurfaceVolume { 

protected:
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


typedef  enum { X=0, Y=1, Z=2 } axis_t;


class CylinderSurface : public SurfaceVolume {

protected:
  axis_t axis;
  double radius;
  Vector3d center;
  bool onaxis;

public:
  CylinderSurface( axis_t axis_p, double radius_p ):
    SurfaceVolume(), axis(axis_p), radius(radius_p), center(origin), onaxis(true)
  {}

  CylinderSurface( axis_t axis_p, double radius_p, double trans1, double trans2 ):
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

    iBase_EntityHandle final_cylinder = embedWithinWorld( positive, igm, world_size, cylinder, true );

    return final_cylinder;
  };

};

#ifdef HAVE_IGEOM_CONE

class ConeSurface : public SurfaceVolume {

protected:

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

  axis_t axis;
  double theta;    /// the cone's opening angle
  Vector3d center; /// the cone's apex 
  bool onaxis;
  enum nappe nappe; 

public:
  ConeSurface( axis_t axis_p, double tsquared_p, double point_p, double nappe_p ):
    SurfaceVolume(), axis(axis_p), theta( atan(sqrt(tsquared_p)) ), center(origin), onaxis(true), nappe(make_nappe(nappe_p))
  {
    center.v[axis] = point_p; 
  }

  ConeSurface( axis_t axis_p, double tsquared_p, Vector3d center_p, double nappe_p ):
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

    iBase_EntityHandle final_cone = embedWithinWorld( positive, igm, world_size, cone, true );

    return final_cone;

    }

};

#endif /* HAVE_IGEOM_CONE */

class TorusSurface : public SurfaceVolume {

protected:
  Vector3d center;
  axis_t axis;
  double radius;
  double ellipse_axis_rad;
  double ellipse_perp_rad;

public:
  TorusSurface( axis_t axis_p, const Vector3d& center_p, double A, double B, double C ) :
    center(center_p), axis( axis_p ), radius(A), ellipse_axis_rad( B ), ellipse_perp_rad( C )
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return center.length() + radius + std::max( ellipse_axis_rad, ellipse_perp_rad );
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    int igm_result;

    iBase_EntityHandle torus;

    iGeom_createTorus( igm, radius, ellipse_perp_rad, &torus, &igm_result );
    CHECK_IGEOM( igm_result, "Creating initial torus");

    if( ellipse_axis_rad != ellipse_perp_rad ){
      double scalef = ellipse_axis_rad / ellipse_perp_rad;
      iGeom_scaleEnt( igm, torus, 1.0, 1.0, scalef, &igm_result );
      CHECK_IGEOM( igm_result, "Scaling torus" );
    }
    
    if( axis == X ){
      iGeom_rotateEnt( igm, torus, 90, 0, 1, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating torus (X)" );
    }
    else if( axis == Y ){
      iGeom_rotateEnt( igm, torus, -90, 1, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating torus (Y)" );
    }

    iGeom_moveEnt( igm, torus, center.v[0], center.v[1], center.v[2], &igm_result);
    CHECK_IGEOM( igm_result, "moving torus to its center point" );
    
    
    iBase_EntityHandle final_torus = embedWithinWorld( positive, igm, world_size, torus, false );

    return final_torus;

  }

};

class SphereSurface : public SurfaceVolume {

protected:
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


    iBase_EntityHandle final_sphere = embedWithinWorld( positive, igm, world_size, sphere, false );
    
    return final_sphere; 
  }

};


class BoxVolume : public SurfaceVolume {

protected:
  Vector3d dimensions;
  Transform transform;
  bool invert;

public:
  BoxVolume( const Vector3d& corner, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3 ) :
    dimensions( v1.length(), v2.length(), v3.length() ), invert(false)
  {
    Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    Vector3d a1 = v1.normalize(), a2 = v2.normalize(), a3 = v3.normalize();
    
    double rot_matrix[9] = 
      { a1.dot(x), a1.dot(y), a1.dot(z),
	a2.dot(x), a2.dot(y), a2.dot(z),
	a3.dot(x), a3.dot(y), a3.dot(z) };

    if( matrix_det(rot_matrix) < 0 ){
      // negative determinant-- we have an improper rotation (rotation + inversion)
      if( OPT_DEBUG ){ std::cout << " inverting box" << std::endl; }
      invert = true;
      for( int i = 0; i < 9; ++i){ rot_matrix[i] = -rot_matrix[i]; }
    }
    
    transform = Transform( rot_matrix, corner );
  }
  
  virtual double getFarthestExtentFromOrigin ( ) const {
    return transform.getTranslation().length() + dimensions.length();
  }
  
protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){


    int igm_result;
    iBase_EntityHandle box;

    iGeom_createBrick( igm, dimensions.v[0], dimensions.v[1], dimensions.v[2], &box, &igm_result );
    CHECK_IGEOM( igm_result, "making box" );

    Vector3d halfdim = dimensions.scale( 1.0 / 2.0 );
    iGeom_moveEnt( igm, box, halfdim.v[0], halfdim.v[1], halfdim.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "moving box (halfdim)" );

    if( transform.hasRot() ){
      const Vector3d& axis = transform.getAxis();
      iGeom_rotateEnt( igm, box, transform.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
      CHECK_IGEOM( igm_result, "rotating box" );
    }
    if( invert ){
      //TODO: ask about the right way to do this.  Rotate seems to work, but I don't really know why...
      iGeom_rotateEnt( igm, box, 180, 0, 0, 0, &igm_result );
      //iGeom_reflectEnt( igm, box, 0, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "inverting box" );
    }

    const Vector3d& v = transform.getTranslation();
    iGeom_moveEnt( igm, box, v.v[0], v.v[1], v.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "moving box (xform)" );

    iBase_EntityHandle final_box = embedWithinWorld( positive, igm, world_size, box, false );

    return final_box;
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
      surface = new CylinderSurface( X, args.at(0) );
    }
    else if( mnemonic == "cy" ){
      surface = new CylinderSurface( Y, args.at(0) );
    }
    else if( mnemonic == "cz" ){
      surface = new CylinderSurface( Z, args.at(0) );
    }
    else if( mnemonic == "c/x"){
      surface = new CylinderSurface( X, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/y"){
      surface = new CylinderSurface( Y, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/z"){
      surface = new CylinderSurface( Z, args.at(2), args.at(0), args.at(1) );
    }
#ifdef HAVE_IGEOM_CONE
    else if( mnemonic == "kx"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( X, args.at(1), args.at(0), arg3 );
    } 
    else if( mnemonic == "ky"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( Y, args.at(1), args.at(0), arg3 );
    }
    else if( mnemonic == "kz"){
      double arg3 = ( args.size() == 3 ? args.at(2) : 0.0 );
      surface = new ConeSurface( Z, args.at(1), args.at(0), arg3 );
    }
    else if( mnemonic == "k/x" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( X, args.at(3), Vector3d(args),  arg5 );
    }
    else if( mnemonic == "k/y" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( Y, args.at(3), Vector3d(args),  arg5 );
    }
    else if( mnemonic == "k/z" ){
      double arg5 = ( args.size() == 5 ? args.at(4) : 0.0 );
      surface = new ConeSurface( Z, args.at(3), Vector3d(args),  arg5 );
    }
#endif /*HAVE_IGEOM_CONE */
    else if( mnemonic == "tx" ){
      surface = new TorusSurface( X, Vector3d(args), args.at(3), args.at(4), args.at(5) );
    } 
    else if( mnemonic == "ty" ){
      surface = new TorusSurface( Y, Vector3d(args), args.at(3), args.at(4), args.at(5) );
    } 
    else if( mnemonic == "tz" ){
      surface = new TorusSurface( Z, Vector3d(args), args.at(3), args.at(4), args.at(5) );
    }
    else if( mnemonic == "box" ){
      surface = new BoxVolume( Vector3d(args), Vector3d(args,3), Vector3d(args,6), Vector3d(args,9) );
    }
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


