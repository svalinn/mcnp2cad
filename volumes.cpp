#include <iostream>
#include <map>
#include <cfloat>

#include "mcnp2cad.hpp"
#include "MCNPInput.hpp"
#include "volumes.hpp"
#include "geometry.hpp"
#include "options.hpp"
#include <armadillo>

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
    //iGeom_rotateEnt( e, 180, 0, 0, 0, &igm_result );

    //if( !t.hasRot() ){
      iGeom_reflectEnt( igm, e, 0, 0, 0, 0, 0, 1, &igm_result );
      iGeom_reflectEnt( igm, e, 0, 0, 0, 0, 1, 0, &igm_result );
      iGeom_reflectEnt( igm, e, 0, 0, 0, 1, 0, 0, &igm_result );
    //}
    //else{
    //  const Vector3d& axis = t.getAxis();
    //  iGeom_reflectEnt( igm, e, axis.v[0], axis.v[1], axis.v[2], &igm_result );
    //}

    //CHECK_IGEOM( igm_result, "inverting for transformation" );
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
  PlaneSurface( const Vector3d& normal_p, const Vector3d& pos, bool end ) :
    SurfaceVolume(), normal(normal_p)
  {
    //This is used to find D in the planar equation Ax + By + Cz - D = 0 when 
    //given two vectors.
    //This comes from the form A(x-a) + B(y-b) + C(z-c) = 0, so D = Aa + Bb + Cc.
    //normal is the vector to which the plane will be perpendicular, pos is to 
    //help determine the position.  If end is true, the plane the end of normal
    //in relation to the end of pos1 will intersect, otherwise the plane and the
    //end of pos will intersect.
    double D;
    if( end ){
      D = normal.v[0]*( pos.v[0] + normal.v[0] ) + normal.v[1]*( pos.v[1] + normal.v[1] ) + normal.v[2]*( pos.v[2] + normal.v[2] );
    }
    else{
      D = normal.v[0]*pos.v[0] + normal.v[1]*pos.v[1] + normal.v[2]*pos.v[2];
    }
    offset = D/normal.length() ;
  }


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

class GeneralQuadraticSurface : public SurfaceVolume {

protected:
  // coefficients of the GQ
  double A_,B_,C_,D_,E_,F_,G_,H_,J_,K_;
  // the cannonical GQ type
  int type;
  // translation from the canoncial GQ to final GQ
  Vector3d translation;
  // rotation matrix from canonical GQ to final GQ
  double rotation_mat[9];
  // principle axes extents of the GQ
  double extents[3];
  // tolerance used to determine
  // if matrix determinant should be considered zero
  const double gq_tol = 1e-8;
  const double equivalence_tol = 1e-06;

  enum GQ_TYPE {UNKNOWN = 0,
               ELLIPSOID,
               ONE_SHEET_HYPERBOLOID,
               TWO_SHEET_HYPERBOLOID,
               ELLIPTIC_CONE,
               ELLIPTIC_PARABOLOID,
               HYPERBOLIC_PARABOLOID,
               ELLIPTIC_CYL,
               HYPERBOLIC_CYL,
               PARABOLIC_CYL};
public:
  GeneralQuadraticSurface(double A, double B, double C, double D, double E, double F, double G, double H, double J, double K):
    SurfaceVolume(),A_(A),B_(B),C_(C),D_(D),E_(E),F_(F),G_(G),H_(H),J_(J),K_(K) {
    //determine canonical form of GQ and determine transformation
    make_canonical();
  }

  virtual double getFarthestExtentFromOrigin() const{ return 0; }
protected:
  void make_canonical()
  {
  //create coefficient matrix
  arma::mat Aa;
  Aa << A_ << D_/2 << F_/2 << arma::endr
     << D_/2 << B_ <<  E_/2 << arma::endr
     << F_/2 << E_/2 << C_ << arma::endr;  
  //create hessian matrix
  arma::mat Ac;
  Ac << A_ << D_/2 << F_/2 << G_/2 << arma::endr
  << D_/2 << B_ << E_/2 <<H_/2 << arma::endr
  << F_/2 << E_/2 << C_ << J_/2 << arma::endr
  << G_/2 <<  H_/2 << J_/2 << K_ << arma::endr;
  
  //characterization values
  int rnkAa, rnkAc, delta, S, D;
  rnkAa = arma::rank(Aa);
  rnkAc = arma::rank(Ac, 1e-16);

  double determinant = arma::det(Ac);
  delta = (determinant < 0) ? -1:1;

  arma::vec eigenvals;
  arma::mat eigenvects;
  arma::eig_sym(eigenvals, eigenvects, Aa);
  arma::vec signs(3);

  for(unsigned int i = 0; i < 3; i++) {
    if (fabs(eigenvals[i]) < gq_tol)
      signs[i] = 1;
    else if (eigenvals[i] > 0)
      signs[i] = 1;
    else if (eigenvals[i] < 0)
      signs[i] = -1;
  }

  S = (fabs(arma::sum(signs)) == 3) ? 1:-1;
  // may need to adjust delta for speical cases using the new scaling factor, K_
  // so we'll calculate that now
  arma:: mat b;
  b << -G_/2 << arma::endr
    << -H_/2 << arma::endr
    << -J_/2 << arma::endr;
  //use Moore-Penrose pseudoinverse to ensure minimal norm least squares solution
  arma::mat Aai = pinv(Aa);
  arma::mat c = Aai*b;
  double dx = c[0], dy = c[1], dz = c[2];
  K_ = K_ + (G_/2)*dx + (H_/2)*dy + (J_/2)*dz;
  if (rnkAa == 2 && rnkAc == 3 && S == 1)
  delta = ((K_ < 0 && signs[0] < 0) || (K_ > 0 && signs[0] > 0)) ? -1:1;
  D = (K_*signs[0]) ? -1:1;
  //set the translation while we're at it
  translation = Vector3d(dx,dy,dz);
  //set the rotaion matrix
  std::copy(eigenvects.memptr(),eigenvects.memptr()+9,rotation_mat);
  //based on characteristic values, get the GQ type
  type = find_type(rnkAa,rnkAc,delta,S,D);
  //set the new canonical values
  A_ = eigenvals[0]; B_ = eigenvals[1]; C_ = eigenvals[2];
  D_ = 0; E_ = 0; F_ = 0;
  G_ = 0; H_ = 0; J_ = 0;
  //K is set above

  // simplify the GQ if possible
  reduce_type();
  record << "GQ Type is: " << type << std::endl;
  }

  // this method reduces a complex GQ to a geometrically equivalent
  // and more CAD-friendly form if appropriate
  void reduce_type() {

    if( ONE_SHEET_HYPERBOLOID == type ) {
      // if the K value is near-zero, reduce to Elliptic Cone
      if ( fabs(K_) < equivalence_tol ) {
	K_ = 0;
	type = ELLIPTIC_CONE;
	return;
      }
    }

    if ( TWO_SHEET_HYPERBOLOID == type ) {
      // if the K value is near-zero, reduce to Elliptic Cone
      if ( fabs(K_) < equivalence_tol ) {
	K_ = 0;
	type = ELLIPTIC_CONE;
	return;
      }
    }

    if ( ELLIPSOID == type ) {
      //if any of the 2nd order terms are near-zero, reduce to Elliptic Cylinder
      if ( fabs(A_) < equivalence_tol ) {
	A_ = 0;
	type = ELLIPTIC_CYL;
	return;
      }	
      else if ( fabs(B_) < equivalence_tol ) {
	B_ = 0;
	type = ELLIPTIC_CYL;
	return;
      }	
      else if ( fabs(C_) < equivalence_tol ) {
	C_ = 0;
	type = ELLIPTIC_CYL;
	return;
      }
    }
    
  };
		   
  GQ_TYPE find_type(int rt, int rf, int del, int s, int d) {
    
    GQ_TYPE t;
    if( 3 == rt && 4 == rf && -1 == del && 1 == s)
      t = ELLIPSOID;
    else if( 3 == rt && 4 == rf && 1 == del && -1 == s)
      t = ONE_SHEET_HYPERBOLOID;
    else if( 3 == rt && 4 == rf && -1 == del && -1 == s)
      t = TWO_SHEET_HYPERBOLOID;
    else if( 3 == rt && 3 == rf && 0 == del && -1 == s)
      t = ELLIPTIC_CONE;
    else if( 2 == rt && 4 == rf && -1 == del && 1 == s)
      t = ELLIPTIC_PARABOLOID;
    else if( 2 == rt && 4 == rf && 1 == del && -1 == s)
      t = HYPERBOLIC_PARABOLOID;
    else if( 2 == rt && 3 == rf && -1 == del && 1 == s)
      t = ELLIPTIC_CYL;
    else if( 2 == rt && 3 == rf && 0 == del && -1 == s)
      t = HYPERBOLIC_CYL;
    else if( 1 == rt && 3 == rf && 0 == del && 1 == s)
      t = PARABOLIC_CYL;
    else
      t = UNKNOWN;

    //special case, replace delta with D
    if( /*2 == rt && 3 == rf && 1 == s && */ d != 0) {
      t = find_type(rt, rf, d, s, 0);
    }
    
    return t;
  }

  iBase_EntityHandle ellipsoid(iGeom_Instance &igm) {
    int igm_result;
    iBase_EntityHandle gq_handle;
    double radius = 1;

    iGeom_createSphere( igm, radius, &gq_handle, &igm_result);
    CHECK_IGEOM( igm_result, "making sphere" );

    iGeom_scaleEnt( igm, gq_handle, 0, 0, 0, sqrt(-K_/A_),sqrt(-K_/B_),sqrt(-K_/C_), &igm_result);
    CHECK_IGEOM( igm_result, "scaling sphere to ellipsoid" );

    return gq_handle;
  }

  iBase_EntityHandle elliptic_cyl(iGeom_Instance &igm, double world_size) {
    int igm_result;
    double r1,r2;
    int axis;
    //figure out which direction is zero
    if (A_ == 0) {
      axis = 0;
      r1 = sqrt(fabs(K_/C_));
      r2 = sqrt(fabs(K_/B_));
    }
    else if (B_ == 0) {
      axis = 1;
      r1 = sqrt(fabs(K_/A_));
      r2 = sqrt(fabs(K_/C_));
    }
    else if (C_ == 0) {
      axis = 2;
      r1 = sqrt(fabs(K_/A_));
      r2 = sqrt(fabs(K_/B_));
    }

    iBase_EntityHandle cyl;
    iGeom_createCylinder(igm,2*world_size,r1,r2,&cyl,&igm_result);
    CHECK_IGEOM(igm_result, "Creating elliptic cylinder for GQ.");

    if (1 == axis) {
      iGeom_rotateEnt(igm,cyl,90,1,0,0,&igm_result);
    }
    else if (0 == axis) {
      iGeom_rotateEnt(igm,cyl,90,0,1,0,&igm_result);
    }
    else if (2 == axis) {
      igm_result = iBase_SUCCESS;
    }
    CHECK_IGEOM(igm_result, "Rotating canonical elliptic cylinder into place.");

    return cyl;
  }

  iBase_EntityHandle elliptic_cone(iGeom_Instance& igm, double world_size) {
    if( 0 == A_ || 0 == B_ || 0 == C_ ){
      if( OPT_DEBUG ){
        record << "Error in GeneralQuadraticSurface::elliptic_cone(double world_size) in volumes.cpp" << std::endl;
        record << "0 == A_ || 0 == B_ || 0 == C_" << std::endl;
      }
      throw std::runtime_error("Error in definition of elliptic cone.");
    }

    iBase_EntityHandle gq_handle;
    int igm_result=0;

    double minor_radius,major_radius,rot_angle;
    int rot_axis;
    //establish orientation
    if (A_ < 0) {
      minor_radius = 2*world_size*sqrt(-A_/C_);
      major_radius = 2*world_size*sqrt(-A_/B_);
      rot_angle = -90;
      rot_axis = 1;
    }
    else if (B_ < 0) { 
      minor_radius = 2*world_size*sqrt(-B_/A_);
      major_radius = 2*world_size*sqrt(-B_/C_);
      rot_angle = 90;
      rot_axis = 0;
    }
    else if (C_ < 0) {
      minor_radius = 2*world_size*sqrt(-C_/A_);
      major_radius = 2*world_size*sqrt(-C_/B_);
      rot_angle = 180;
      rot_axis = 0;
    }

    //create cone
    iBase_EntityHandle pos_cone;
    iGeom_createCone( igm, 2*world_size, major_radius, minor_radius, 0, &pos_cone, &igm_result);
    CHECK_IGEOM(igm_result, "Creating positive cone for GQ.");

    //now move the cone s.t. the point is on the origin
    iGeom_moveEnt( igm, pos_cone, 0, 0, -world_size, &igm_result );
    CHECK_IGEOM(igm_result, "Moving positive cone for GQ.");

    double rot_vec[3] = {0,0,0};
    rot_vec[rot_axis] = 1;

    //rotate to proper axis
    iGeom_rotateEnt( igm, pos_cone, rot_angle, rot_vec[0], rot_vec[1], rot_vec[2], &igm_result );
    CHECK_IGEOM(igm_result, "Rotating positive cone for GQ.");

    //create a copy
    iBase_EntityHandle neg_cone;
    iGeom_copyEnt( igm, pos_cone, &neg_cone, &igm_result );
    CHECK_IGEOM(igm_result, "Copying positive cone for GQ.");

    iGeom_rotateEnt( igm, neg_cone, 180, rot_vec[0], rot_vec[1], rot_vec[2], &igm_result );
    CHECK_IGEOM(igm_result, "Rotating negative cone for GQ.");

    iBase_EntityHandle cones[2] = {pos_cone, neg_cone};
    iGeom_uniteEnts( igm, cones, 2, &gq_handle, &igm_result );
    CHECK_IGEOM(igm_result, "Uniting positive and negative cones for GQ.");

    return gq_handle;
  }

  virtual iBase_EntityHandle getHandle(bool positive, iGeom_Instance& igm, double world_size) {

    iBase_EntityHandle gq;
    switch(type){
    case ELLIPSOID:
      gq = ellipsoid(igm);
      break;
    case ELLIPTIC_CONE:
      gq = elliptic_cone( igm, world_size);
      break;
    case ELLIPTIC_CYL:
      gq = elliptic_cyl( igm, world_size);
      break;
    default:
      record << "GQ type is currently unsupported" << std::endl;
    }

    //re-orient gq into original position
    Transform rotation_transform(rotation_mat, Vector3d(0,0,0));    
    applyReverseTransform( rotation_transform, igm, gq);
    Transform translation_transform(translation);    
    applyTransform(translation_transform, igm, gq);

    iBase_EntityHandle final_gq = embedWithinWorld(-positive, igm, world_size, gq, true);
    return final_gq;
  }

};

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
    iGeom_createCylinder( igm,  2.0 * world_size, radius, 0, &cylinder, &igm_result);
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

    int wrapper = static_cast<int>(param);
    enum nappe n = static_cast<enum nappe>(wrapper);
    //enum nappe n = static_cast<enum nappe>(param);
    if( -1 <= n && n <= 1 ){
      return n;
    }
    else{
      std::cerr << "WARNING: Bad cylinder +/-1 argument: " << param << std::endl;
      std::cerr << "         will pretend it was really 0" << std::endl;
      record << "WARNING: Bad cylinder +/-1 argument: " << param << std::endl;
      record << "         will pretend it was really 0" << std::endl;
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
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm,double world_size ){

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
      double scalef = fabs(ellipse_axis_rad) / fabs(ellipse_perp_rad);
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

class EllipsoidSurface : public SurfaceVolume {

protected:
  Vector3d center;
  Vector3d axes;

public:
  EllipsoidSurface( const Vector3d& center_p, const Vector3d& axes_p ) :
    SurfaceVolume(), center(center_p), axes(axes_p)
  {}

  virtual ~EllipsoidSurface(){}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return (center.length() + axes.length());
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    int igm_result;
    iBase_EntityHandle sphere;
    double radius = 1;

    iGeom_createSphere( igm, radius, &sphere, &igm_result);
    CHECK_IGEOM( igm_result, "making sphere" );

    iGeom_scaleEnt( igm, sphere, 0, 0, 0, sqrt(1/axes.v[0]), sqrt(1/axes.v[1]), sqrt(1/axes.v[2]), &igm_result);
    CHECK_IGEOM( igm_result, "scaling sphere to ellipsoid" );

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

  if( OPT_DEBUG ) record << "Axes image: " << a1 << " : " << a2 << " : " << a3 << std::endl;

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
    if( OPT_DEBUG ) record << "imageZAxisTo: Changing v " << std::endl;
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
  bool facet;

public:
  RecVolume( const Vector3d& center_p, const Vector3d& axis, const Vector3d& v1, const Vector3d& v2, bool facet_p ) :
    base_center( center_p ), transform( axesImage( v1, v2, axis, center_p ) ), 
    length( axis.length() ), radius1( v1.length() ), radius2( v2.length() ), facet( facet_p)
  {}

  RecVolume( const Vector3d& center_p, const Vector3d& axis, const Vector3d& v1, double length2, bool facet_p ) :
    base_center( center_p ), transform( axesImage( v1, v1.cross(axis), axis, center_p ) ), 
    length( axis.length() ), radius1( v1.length() ), radius2( length2 ), facet( facet_p )
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return base_center.length() + length + std::max( radius1, radius2 );
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle rec;

    if( facet ){
      iGeom_createCylinder( igm, 2.0 * world_size + length / 2.0, radius1, radius2, &rec, &igm_result );
    }
    else{
      iGeom_createCylinder( igm, length, radius1, radius2, &rec, &igm_result );
    }
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
  bool facet;

public:
  RccVolume( const Vector3d& center_p, const Vector3d& axis, double radius_p, bool facet_p ) :
    base_center( center_p ), transform( imageZAxisTo( axis, center_p ) ), length( axis.length() ), radius(radius_p), facet( facet_p ) 
  {}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return base_center.length() + length + radius;
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;
    iBase_EntityHandle rcc;

    if( facet ){
      iGeom_createCylinder( igm, 2.0 * world_size, radius, 0, &rcc, &igm_result );
    }
    else{
      iGeom_createCylinder( igm, length, radius, 0, &rcc, &igm_result );
    }
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
      record << "Inferred vectors for 9-args HEX/RHP:" << RV << SV << TV << std::endl;
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
    if( !contains( c ) ){
      if( OPT_DEBUG ){
        record << "Error in VolumeCache::get( const SurfaceCard* c) in volumes.cpp" << std::endl;
        record << "!contains( c )" << std::endl;
      }
      throw std::runtime_error("Surface card not found in surface volume.");
    }
    return (*(mapping.find(c))).second;
  }

  void insert( const SurfaceCard* c, SurfaceVolume* s ){
    mapping[c] = s;
  }

};

bool sqIsEllipsoid(const std::vector< double >& args) {

  bool isEllipsoid = true;

  if ( args.at(3)*args.at(3) + args.at(4)*args.at(4) + args.at(5)*args.at(5) != 0)
    isEllipsoid = false;

  if ( args.at(0)*args.at(1)*args.at(2)*args.at(6) >= 0 )
    isEllipsoid = false;

  return isEllipsoid;

}

// the VolumeCache to use if none is provided to makeSurface() calls.
static VolumeCache default_volume_cache;

// Facet number is 0 if not provided
SurfaceVolume& makeSurface( const SurfaceCard* card, VolumeCache* v, int facet){
  VolumeCache& cache = default_volume_cache;
  SurfaceVolume* surface;
  if( v != NULL ){
    cache = *v;
  }


  if( cache.contains( card ) ){
    return *cache.get( card );
  }
  else if(facet != 0 ){ 
    // SurfaceCard variables:  mnemonic, args, coord_xform  
    const std::string& mnemonic = card->getMnemonic();
    const std::vector< double >& args = card->getArgs(); 
    // special function for macrobody facets
    surface = FacetSurface( mnemonic, args, facet );
  }

  else{
    // SurfaceCard variables:  mnemonic, args, coord_xform  
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
    else if( mnemonic == "sq" && sqIsEllipsoid(args) ){
      surface = new EllipsoidSurface( Vector3d( args.at(7), args.at(8), args.at(9) ), Vector3d( args )* (-1/args.at(6)) );
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
      surface = new RccVolume( Vector3d(args), Vector3d(args,3), args.at(6), false );
    }
    else if( mnemonic == "rec" ){
      if( args.size() == 10 ){
        surface = new RecVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6),  args.at(9), false );
      }
      else{
        surface = new RecVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6), Vector3d(args,9), false );
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
    else if ( mnemonic == "gq" )
      surface = new GeneralQuadraticSurface(args.at(0), args.at(1), args.at(2), args.at(3), args.at(4), args.at(5), args.at(6), args.at(7), args.at(8), args.at(9));
    else{
      throw std::runtime_error( mnemonic + " is not a supported surface" );
    }
  } 
  if( card->getTransform().hasData() ){
    const Transform& transform = card->getTransform().getData();
    surface->setTransform( &transform );
  }

  cache.insert( card, surface );
  return *surface;

}


SurfaceVolume* FacetSurface( const std::string mnemonic, const std::vector< double > args, int facet ){
  if( mnemonic == "rcc" ){
    return rccFacet( args, facet );
  }
  else if(mnemonic == "box"){
    return boxFacet( args, facet );
  }
  else if( mnemonic == "rpp"){
    return rppFacet( args, facet );
  }
  else if( mnemonic == "hex" || mnemonic == "rhp" ){
    return hexFacet( args, facet );
  }
  else if( mnemonic == "rec" ){
    return recFacet( args, facet );
  }
  throw std::runtime_error( mnemonic + " does not have macrobody facet support at this time." );
}    



SurfaceVolume* rccFacet( const std::vector< double > args, int facet ){
  if( facet == 1 ){
    //cylinder surface
    return new RccVolume( Vector3d(args), Vector3d(args,3), args.at(6), true );
  }
  else if( facet == 2 || facet == 3 ){
    //the two end planes
    Vector3d v1( args, 3 );
    Vector3d v2( args, 0 );
    return new PlaneSurface( v1, v2, facet == 2 );
  }
  else{
    throw std::runtime_error( "rcc only has 3 facets" );
  }
}

SurfaceVolume* boxFacet( const std::vector< double > args, int facet ){
  //This function creates the planes for the facets of box.
  if( facet >= 1 && facet <= 6 ){
    //if the facet number is odd, the plane is at the end of the vector specified
    //which consists of arguments 3,4,5 for 1 & 2, 6,7,8 for 3 & 4, and so on.
    int direction = ( facet - 1 )/2;
    Vector3d v1( args, ( direction + 1 )*3 );
    Vector3d v2( args, 0 );
    return new PlaneSurface( v1, v2, facet%2 );
  }
  else{
    throw std::runtime_error( "box only has 6 facets" );
  }
}

SurfaceVolume* rppFacet( const std::vector< double > args, int facet ){
  if( facet >= 1 && facet <= 6 ){
    //The vector3d determines which plane; 1 and 2 becomes (1,0,0), 3 and 4 (0,1,0), etc
    Vector3d v;
    int direction = ( facet - 1 )/2;
    v.v[direction] = 1;
    //The assignment of the facets is kind of backwards given the ordering of the arguments,
    //so there needs to be interesting math to get 1 0 3 2 5 4.
    return new PlaneSurface( v, args.at( 2*direction + facet%2 ) );
  }
  else{
    throw std::runtime_error( "rpp only has 6 facets." );
  }
}

SurfaceVolume* hexFacet( const std::vector< double > args, int facet ){
  if( args.size() == 9 && facet <= 6 && facet >= 3 ){
    //These facets go 1 3 5 2 4 6 counter clockwise, with 1 and 2 as they are for 12 argument version.
    Vector3d v1( args, 6 );
    Vector3d v2( args, 3 );
    Vector3d v3( args, 0 );
    //even vectors are opposite the odd vector one less than them; thus the scaling.
    Vector3d v( v1.rotate_about( v2, 60*( ( facet - 1 )/2) ).scale( -1.0 + 2*(facet%2) ) );
    return new PlaneSurface( v, v3, true );
  }
  else if( facet >= 1 && facet <= 6 ){
    //Similar to the box geometry, except that the even facets go in the opposite direction, instead
    //of being at the point of origin of the second vector.
    int direction = ( facet + facet%2 )/2;
    Vector3d v1( args, ( direction + 1 )*3 );
    Vector3d v2( args, 0 );
    return new PlaneSurface( v1.scale( -1.0 + 2*(facet%2) ), v2, true );
  }
  else if( facet == 7 || facet == 8 ){
    Vector3d v1( args, 3 );
    Vector3d v2( args, 0 );
    return new PlaneSurface( v1, v2, facet == 7 );
  }
  else{
    throw std::runtime_error( "hex and rhp only have 8 facets." );
  }
}


SurfaceVolume* recFacet( const std::vector< double > args, int facet ){
  if( facet == 1 ){
    //elliptical cylinder surface
    if( args.size() == 10 ){
      return new RecVolume( Vector3d( args ), Vector3d(args, 3), Vector3d(args,6), args.at(9), true );
    }
    else{
      return new RecVolume( Vector3d( args ), Vector3d(args,3), Vector3d(args,6), Vector3d(args,9).length(), true );
    }
  }    
  else if( facet == 2 || facet == 3 ){
    Vector3d v1( args, 3 );
    Vector3d v2( args, 0 );
    return new PlaneSurface( v1, v2, facet == 2 );
  }
  else{
    throw std::runtime_error( "rec only has 3 facets." );
  }
}

