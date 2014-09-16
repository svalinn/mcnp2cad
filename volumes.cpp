#include <iostream>
#include <map>
#include <cfloat>

#include <cassert>

#include "MCNPInput.hpp"
#include "volumes.hpp"
#include "geometry.hpp"
#include "options.hpp"


static Vector3d origin(0,0,0);


iBase_EntityHandle makeWorldSphere( iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle world_sphere;
  int igm_result;
  // Note: I tried using createBrick instead of createSphere to bound the universe with a box
  // instead of a sphere.  This worked but led to a substantial increase in run times and
  // memory usage, so should be avoided.
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



iBase_EntityHandle applyTransform( const Transform& t, iGeom_Instance& igm, iBase_EntityHandle& e ) {
  
  int igm_result;
  if( t.hasRot()  ){
    const Vector3d& axis = t.getAxis();
    iGeom_rotateEnt( igm, e, t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "applying rotation" );
  }
  
  if( t.hasInversion() ){
      //TODO: ask about the right way to do this.  Rotate seems to work, but I don't really know why...
      //iGeom_rotateEnt( igm, e, 180, 0, 0, 0, &igm_result );

    //if( !t.hasRot() ){
      iGeom_reflectEnt( igm, e, 0, 0, 0, 0, 0, 1, &igm_result );
      iGeom_reflectEnt( igm, e, 0, 0, 0, 0, 1, 0, &igm_result );
      iGeom_reflectEnt( igm, e, 0, 0, 0, 1, 0, 0, &igm_result );
      //}
      //else{
      //   const Vector3d& axis = t.getAxis();
      //   iGeom_reflectEnt( igm, e, axis.v[0], axis.v[1], axis.v[2], &igm_result );
      //   }

      CHECK_IGEOM( igm_result, "inverting for transformation" );
    }


  const Vector3d& translation = t.getTranslation();
  iGeom_moveEnt( igm, e, translation.v[0], translation.v[1], translation.v[2], &igm_result);
  CHECK_IGEOM( igm_result, "applying translation" );
  
  return e;
}

iBase_EntityHandle applyReverseTransform( const Transform& tx, iGeom_Instance& igm, iBase_EntityHandle& e ) {
  
  int igm_result;
  Transform rev_t = tx.reverse();

  const Vector3d& translation = rev_t.getTranslation();
  iGeom_moveEnt( igm, e, translation.v[0], translation.v[1], translation.v[2], &igm_result);
  CHECK_IGEOM( igm_result, "applying reverse translation" );

  if( rev_t.hasInversion() ){
    iGeom_rotateEnt( igm, e, 180, 0, 0, 0, &igm_result );
    CHECK_IGEOM( igm_result, "inverting for reverse transformation" );
  }

  if( rev_t.hasRot() ){
    const Vector3d& axis = rev_t.getAxis();
    iGeom_rotateEnt( igm, e, rev_t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "applying rotation" );
  }
  
  
  return e;
}


iBase_EntityHandle SurfaceVolume::define( bool positive, iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle handle = this->getHandle( positive, igm, world_size );
  if( transform ){
    handle = applyTransform( *transform, igm, handle );
  }
  return handle;
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
    return sqrt(3.0) *  std::fabs(offset);
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

    // based on the textual descriptions in the manual, I think the following expression should be 
    // height * tan ( theta / 2 ) -- unless "opening angle" refers to only half the apex angle
    // of the cylinder.  But this implementation seems to be more correct in examples I can check against.
    double base_radius = height * tan( theta );

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
      iGeom_scaleEnt( igm, torus, 0, 0, 0, 1.0, 1.0, scalef, &igm_result );
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



static Transform axesImage( const Vector3d& v1, const Vector3d& v2, const Vector3d &v3, const Vector3d& translation = Vector3d() )
{
  Vector3d x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
  Vector3d a1 = v1.normalize(), a2 = v2.normalize(), a3 = v3.normalize();
    
  if( OPT_DEBUG ) std::cout << "Axes image: " << a1 << " : " << a2 << " : " << a3 << std::endl;

  double rot_matrix[9] = 
    { a1.dot(x), a2.dot(x), a3.dot(x),
      a1.dot(y), a2.dot(y), a3.dot(y),
      a1.dot(z), a2.dot(z), a3.dot(z) };
  

  return Transform( rot_matrix, translation );
}

static Transform imageZAxisTo( const Vector3d& v, const Vector3d& translation = Vector3d() ){
  // approach: find two vectors perpendicular to v, then use axesImage.
  Vector3d a = v.normalize();

  Vector3d x(1, 0, 0);
  Vector3d b = x.cross( v.normalize() );
  
  if( b.length() < DBL_EPSILON ){
    // v is indistinguishable from the x axis
    b = Vector3d(0, -1, 0);
    if( OPT_DEBUG ) std::cout << "imageZAxisTo: Changing v " << std::endl;
  }

  Vector3d c = b.cross( v.normalize() );
  return axesImage( c, b, a, translation );
  

}



class BoxVolume : public SurfaceVolume {

protected:
  Vector3d dimensions;
  Transform transform;

public:
  BoxVolume( const Vector3d& corner, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3 ) :
    dimensions( v1.length(), v2.length(), v3.length() ), transform( axesImage(v1,v2,v3,corner) )
  {}
  
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

    box = applyTransform( transform, igm, box );
    iBase_EntityHandle final_box = embedWithinWorld( positive, igm, world_size, box, false );

    return final_box;
  }

};


class RppVolume : public SurfaceVolume {

protected:
  Vector3d dimensions;
  Vector3d center_offset;

public:
  RppVolume( const Vector3d& lower, const Vector3d& upper ) 
  {
    for( int i = 0; i < 3; ++i ){
      dimensions.v[i]    = upper.v[i] - lower.v[i];
      center_offset.v[i] = (upper.v[i]+lower.v[i]) / 2.0;
    }
  }
  
  virtual double getFarthestExtentFromOrigin ( ) const {
    return dimensions.length() + center_offset.length();
  }
  
protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    int igm_result;
    iBase_EntityHandle rpp;

    iGeom_createBrick( igm, dimensions.v[0], dimensions.v[1], dimensions.v[2], &rpp, &igm_result );
    CHECK_IGEOM( igm_result, "making rpp" );

    iGeom_moveEnt( igm, rpp, center_offset.v[0], center_offset.v[1], center_offset.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "moving rpp" );

    iBase_EntityHandle final_rpp = embedWithinWorld( positive, igm, world_size, rpp, false );
    return final_rpp;
  }

};


class RecVolume : public SurfaceVolume { 

protected:
  Vector3d base_center;
  Transform transform;
  double length, radius1, radius2;

public:
  RecVolume( const Vector3d& center_p, const Vector3d& axis, const Vector3d& v1, const Vector3d& v2 ) :
    base_center( center_p ), transform( axesImage( v1, v2, axis, center_p ) ), 
    length( axis.length() ), radius1( v1.length() ), radius2( v2.length() )
  {}

  RecVolume( const Vector3d& center_p, const Vector3d& axis, const Vector3d& v1, double length2 ) :
    base_center( center_p ), transform( axesImage( v1, v1.cross(axis), axis, center_p ) ), 
    length( axis.length() ), radius1( v1.length() ), radius2( length2 )
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return base_center.length() + length + std::max( radius1, radius2 );
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle rec;
    
    iGeom_createCylinder( igm, length, radius1, radius2, &rec, &igm_result );
    CHECK_IGEOM( igm_result, "creating rec" );
    

    double movement_factor = length / 2.0;
    iGeom_moveEnt( igm, rec, 0, 0, movement_factor, &igm_result );
    CHECK_IGEOM( igm_result, "moving rec" );
    
    rec = applyTransform( transform, igm, rec );
    iBase_EntityHandle final_rec = embedWithinWorld( positive, igm, world_size, rec, false );
    return final_rec;
  }
};


class RccVolume : public SurfaceVolume {

protected:
  Vector3d base_center;
  Transform transform;
  double length, radius;

public:
  RccVolume( const Vector3d& center_p, const Vector3d& axis, double radius_p ) :
    base_center( center_p ), transform( imageZAxisTo( axis, center_p ) ), length( axis.length() ), radius(radius_p) 
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return base_center.length() + length + radius;
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle rcc;

    iGeom_createCylinder( igm, length, radius, 0, &rcc, &igm_result );
    CHECK_IGEOM( igm_result, "creating rcc" );
    
    double movement_factor = length / 2.0;
    iGeom_moveEnt( igm, rcc, 0, 0, movement_factor, &igm_result );
    CHECK_IGEOM( igm_result, "moving rcc" );

    rcc = applyTransform( transform, igm, rcc );
    iBase_EntityHandle final_rcc = embedWithinWorld( positive, igm, world_size, rcc, false );
    return final_rcc;
  }

};


#ifdef HAVE_IGEOM_CONE
class TrcVolume : public SurfaceVolume { 

protected:
  Vector3d base_center;
  Transform transform;
  double length, radius1, radius2;

public:
  TrcVolume( const Vector3d& center_p, const Vector3d& axis, double radius1_p, double radius2_p ) :
    base_center( center_p ), transform( imageZAxisTo( axis, center_p ) ), length( axis.length() ), 
    radius1(radius1_p), radius2(radius2_p) 
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return base_center.length() + length + std::max(radius1,radius2);
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle trc;

    iGeom_createCone( igm, length, radius1, 0, radius2, &trc, &igm_result );
    CHECK_IGEOM( igm_result, "creating trc" );
    
    iGeom_moveEnt( igm, trc, 0, 0, length / 2.0, &igm_result );
    CHECK_IGEOM( igm_result, "moving trc" );

    trc = applyTransform( transform, igm, trc );
    iBase_EntityHandle final_trc = embedWithinWorld( positive, igm, world_size, trc, false );
    return final_trc;
  }

};
#endif


class HexVolume : public SurfaceVolume { 

protected:
  Vector3d base_center;
  Vector3d heightV, RV, SV, TV; 
  //Transform transform;

public:
  HexVolume( const Vector3d& center_p, const Vector3d& h_p, const Vector3d& r_p, 
             const Vector3d& s_p, const Vector3d& t_p ) :
    base_center(center_p), heightV(h_p), RV(r_p), SV(s_p), TV(t_p)
  {}

  HexVolume( const Vector3d& center_p, const Vector3d& h_p, const Vector3d& r_p ) :
    base_center(center_p), heightV(h_p), RV(r_p), SV( r_p.rotate_about(h_p,60.0) ), TV( r_p.rotate_about(h_p,120.0) )
  {
    if( OPT_DEBUG ){ 
      std::cout << "Inferred vectors for 9-args HEX/RHP:" << RV << SV << TV << std::endl;
    }
  }

  virtual double getFarthestExtentFromOrigin ( ) const { 
    double hex_max = std::max( RV.length(), std::max( SV.length(), TV.length() ) );
    return base_center.length() + heightV.length() + hex_max; 
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle hex;

    hex = makeWorldSphere( igm, world_size );

    Vector3d b = - heightV.normalize();
    iGeom_sectionEnt( igm, hex, b.v[0], b.v[1], b.v[2], 0, true, &hex, &igm_result );
    CHECK_IGEOM( igm_result, "Sectioning world for a hex (1)" );

    b = -b;
    iGeom_sectionEnt( igm, hex, b.v[0], b.v[1], b.v[2], heightV.length(), true, &hex, &igm_result );
    CHECK_IGEOM( igm_result, "Sectioning world for a hex (2)" );


    
    const Vector3d* vec[3] = {&RV, &SV, &TV};
    for( int i = 0; i < 3; ++i ){
      Vector3d v = *(vec[i]);
      double length = v.length(); 
      v = v.normalize();
      
      iGeom_sectionEnt( igm, hex, v.v[0], v.v[1], v.v[2], length, true, &hex, &igm_result );
      CHECK_IGEOM( igm_result, "Sectioning world for a hex (3)" );
      
      v = -v;
      iGeom_sectionEnt( igm, hex, v.v[0], v.v[1], v.v[2], length, true, &hex, &igm_result );
      CHECK_IGEOM( igm_result, "Sectioning world for a hex (4)" );
      

    }

    iGeom_moveEnt( igm, hex, base_center.v[0], base_center.v[1], base_center.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "Moving hex" );

    iBase_EntityHandle final_hex = embedWithinWorld( positive, igm, world_size, hex, false );
    return final_hex;

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
      if( args.size() == 4 ){
        // plane given as ABCD coefficients (Ax+By+Cz-D=0)
        Vector3d v(args);
        surface = new PlaneSurface( v, args.at(3)/v.length() );
      }
      else if( args.size() == 9 ){
        // plane is given at three points in space
        Vector3d v1(args), v2(args,3), v3(args,6);

        // ordering of the points is arbitrary, but make sure v1 is furthest from origin.
        if( v1.length() < v2.length() ){
          std::swap( v1, v2 );
        }
        if( v1.length() < v3.length() ){
          std::swap( v1, v3 );
        }

        // the normal (up to reversal) of the plane 
        // The normal may need to be reversed to ensure that the origin
        // has negative sense, as required by MCNP
        Vector3d normal = v1.add(-v2).cross( v1.add(-v3) ).normalize();

        // If a ray started from the origin and followed the normal vector,
        // p is the point where it intersects the plane
        Vector3d p = normal.scale( v1.dot(normal) );

        // cos of the angle between v1 and normal 
        double angle = normal.dot( v1.normalize() );
        
        // invert the normal if the angle is > 90 degrees, which indicates
        // that reversal is required.
        if( angle < 0 ){
          normal   = -normal;
        }

        //std::cout << normal << " " << p.length() << " : " << angle << std::endl;
        surface = new PlaneSurface( normal, p.length() );


      }
      else{ 
        throw std::runtime_error( "P surface with unsupported number of params" );
      }

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
    else if( mnemonic == "trc" ){
      surface = new TrcVolume( Vector3d(args), Vector3d(args,3), args.at(6), args.at(7) );
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
    else if( mnemonic == "rpp" ){
      surface = new RppVolume( Vector3d(args.at(0), args.at(2), args.at(4)), Vector3d(args.at(1), args.at(3), args.at(5)) );
    }
    else if( mnemonic == "rcc" ){
      surface = new RccVolume( Vector3d(args), Vector3d(args,3), args.at(6) );
    }
    else if( mnemonic == "rec" ){
      if( args.size() == 10 ){
        surface = new RecVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6),  args.at(9) );
      }
      else{
        surface = new RecVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6), Vector3d(args,9) );
      }
    }
    else if( mnemonic == "hex" || mnemonic == "rhp" ){
      if( args.size() == 9 ){
        surface = new HexVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6) );
      }
      else{
        surface = new HexVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6), Vector3d(args,9), Vector3d(args,12) );
      }
    }
    else if( mnemonic == "x" ) {
      switch (args.size()) {
      case 2: // plane
        surface = new PlaneSurface( Vector3d( 1, 0, 0), args.at(0) );
        break;
      case 4: // either plane, cylinder or cone
        if ( args.at(0) == args.at(2) ) // plane
          surface = new PlaneSurface( Vector3d( 1, 0, 0), args.at(0) );
        else if (args.at(1) == args.at(3)) // cylinder
          surface = new CylinderSurface( X, args.at(1) );
        else // cone
          {
            double m = (args.at(3) - args.at(1))/(args.at(2)-args.at(0));
            double apex_p = args.at(0) - args.at(1)/m;
            surface = new ConeSurface( X, m*m, apex_p, (m > 0 ? 1 : -1 ) );
          }
        break;
      default:
        throw std::runtime_error( mnemonic + " is only a supported surface with 2 or 4 arguments" );
        break;
      }
    }
    else if( mnemonic == "y" ) {
      switch (args.size()) {
      case 2: // plane
        surface = new PlaneSurface( Vector3d( 0, 1, 0), args.at(0) );
        break;
      case 4: // either plane, cylinder or cone
        if ( args.at(0) == args.at(2) ) // plane
          surface = new PlaneSurface( Vector3d( 0, 1, 0), args.at(0) );
        else if (args.at(1) == args.at(3)) // cylinder
          surface = new CylinderSurface( Y, args.at(1) );
        else // cone
          {
            double m = (args.at(3) - args.at(1))/(args.at(2)-args.at(0));
            double apex_p = args.at(0) - args.at(1)/m;
            surface = new ConeSurface( Y, m*m, apex_p, (m > 0 ? 1 : -1 ) );
          }
        break;
      default:
        throw std::runtime_error( mnemonic + " is only a supported surface with 2 or 4 arguments" );
        break;
      }
    }
    else if( mnemonic == "z" ) {
      switch (args.size()) {
      case 2: // plane
        surface = new PlaneSurface( Vector3d( 0, 0, 1), args.at(0) );
        break;
      case 4: // either plane, cylinder or cone
        if ( args.at(0) == args.at(2) ) // plane
          surface = new PlaneSurface( Vector3d( 0, 0, 1), args.at(0) );
        else if (args.at(1) == args.at(3)) // cylinder
          surface = new CylinderSurface( Z, args.at(1) );
        else // cone
          {
            double m = (args.at(3) - args.at(1))/(args.at(2)-args.at(0));
            double apex_p = args.at(0) - args.at(1)/m;
            surface = new ConeSurface( Z, m*m, apex_p, (m > 0 ? 1 : -1 ) );
          }
        break;
      default:
        throw std::runtime_error( mnemonic + " is only a supported surface with 2 or 4 arguments" );
        break;
      }
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


