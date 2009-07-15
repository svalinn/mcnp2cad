#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <cmath>
#include <map>

#include <cassert>

//#include "MCNPInputDeck.hpp"
#include "iGeom.h"
#include "geometry.hpp"
#include "MCNPInput.hpp"


class CellCard;
class SurfaceCard;
class DataCard;
class LineExtractor;
class InputDeck;



class Transform;

class AbstractSurface{

protected:
  const Transform* transform;

public:
  AbstractSurface( const Transform* transform_p = NULL):
    transform(transform_p)
  {}
  virtual ~AbstractSurface(){}

  void setTransform( const Transform* transform_p ){ transform = transform_p; }
  
  virtual double getFarthestExtentFromOrigin( ) const = 0;
  virtual iBase_EntityHandle define( bool positive, iGeom_Instance& igm, double universe_size );

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ) = 0;
};


#define CHECK_IGEOM(err, msg) \
  do{if((err) != iBase_SUCCESS) std::cerr << "iGeom error (" << err << "): " << msg << std::endl; }while(0)

iBase_EntityHandle applyTransform( const Transform& t, iGeom_Instance& igm, iBase_EntityHandle& e ) {
  
  int igm_result;
  if( t.hasRot() ){
    iGeom_rotateEnt( igm, &e, t.getRotX(), 1, 0, 0, &igm_result );
    CHECK_IGEOM( igm_result, "applying x rotation" );
    
    iGeom_rotateEnt( igm, &e, t.getRotY(), 0, 1, 0, &igm_result );
    CHECK_IGEOM( igm_result, "applying y rotation" );
    
    iGeom_rotateEnt( igm, &e, t.getRotZ(), 0, 0, 1, &igm_result );
    CHECK_IGEOM( igm_result, "applying z rotation" );
  }
  
  const Vector3d& translation = t.getTranslation();
  iGeom_moveEnt( igm, &e, translation.v[0], translation.v[1], translation.v[2], &igm_result);
  CHECK_IGEOM( igm_result, "applying translation" );
  
  return e;
}
static Vector3d origin(0,0,0);


iBase_EntityHandle makeUniverseSphere( iGeom_Instance& igm, double universe_size ){
  iBase_EntityHandle universe_sphere;
  int igm_result;
  iGeom_createSphere( igm, universe_size, &universe_sphere, &igm_result);
  CHECK_IGEOM( igm_result, "making universe sphere" );
  return universe_sphere;
}

iBase_EntityHandle AbstractSurface::define( bool positive, iGeom_Instance& igm, double universe_size ){
  iBase_EntityHandle handle = this->getHandle( positive, igm, universe_size );
  if( transform ){
    handle = applyTransform( *transform, igm, handle );
  }
  return handle;
}

class PlaneSurface : public AbstractSurface { 

public:
  Vector3d normal;
  double offset;

public:
  PlaneSurface( const Vector3d& normal_p, double offset_p ) :
    AbstractSurface(), normal(normal_p), offset(offset_p) 
  {}
  
  virtual double getFarthestExtentFromOrigin() const{
    // this is a funny situation, since planes are technically infinte...
    // in order to have a sane answer, we just return the offset from the origin.
    // (multiplied by root 3, which was done in the old converter, why?)
    return sqrt(3.0) *  std::abs(offset);
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size){

    int igm_result;
    iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);
    iBase_EntityHandle hemisphere;
    // note the reversal of sense in this call; mcnp and igeom define it differently.
    iGeom_sectionEnt( igm, &universe_sphere, 
		      normal.v[0], normal.v[1], normal.v[2], offset, !positive, &hemisphere, &igm_result);
    CHECK_IGEOM( igm_result, "Sectioning universe for a plane" );
    return hemisphere;


  }

};

class CylinderSurface : public AbstractSurface {

public:
  enum axis{ X=0, Y=1, Z=2 } axis;
  double radius;
  Vector3d center;
  bool onaxis;

  CylinderSurface( enum axis axis_p, double radius_p ):
    AbstractSurface(), axis(axis_p), radius(radius_p), center(origin), onaxis(true)
  {}

  CylinderSurface( enum axis axis_p, double radius_p, double trans1, double trans2 ):
    AbstractSurface(), axis(axis_p), radius(radius_p), center(origin), onaxis(false)
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
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ){
    int igm_result;

    iBase_EntityHandle cylinder;
    iGeom_createCylinder( igm, 2.0 * universe_size, radius, 0, &cylinder, &igm_result);
    CHECK_IGEOM( igm_result, "making cylinder" );

    
    if( axis == X ){
      iGeom_rotateEnt( igm, &cylinder, 90, 0, 1, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (X)" );
    }
    else if( axis == Y ){
      iGeom_rotateEnt( igm, &cylinder, 90, 1, 0, 0, &igm_result );
      CHECK_IGEOM( igm_result, "rotating cylinder (Y)" );
    }

    if( onaxis == false ){
      iGeom_moveEnt( igm, &cylinder, center.v[0], center.v[1], center.v[2], &igm_result);
      CHECK_IGEOM( igm_result, "moving cylinder" );
    }

    iBase_EntityHandle universe_sphere = makeUniverseSphere( igm, universe_size );
    iBase_EntityHandle final_cylinder;

    if( positive ){
      iGeom_subtractEnts( igm, universe_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making clipped cylinder" );
    }
    else{
      iGeom_intersectEnts( igm, universe_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making negative cylinder" );
    }

    return final_cylinder;
  };

};

class SphereSurface : public AbstractSurface {

public:
  Vector3d center;
  double radius;

public:
  SphereSurface( const Vector3d& center_p, double radius_p ) :
    AbstractSurface(), center(center_p), radius(radius_p)
  {}

  virtual ~SphereSurface(){}

  virtual double getFarthestExtentFromOrigin ( ) const {
    return (center.length() + radius);
  }

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double universe_size ){

    int igm_result;
    iBase_EntityHandle sphere;

    iGeom_createSphere( igm, radius, &sphere, &igm_result);
    CHECK_IGEOM( igm_result, "making sphere" );

    iGeom_moveEnt( igm, &sphere, center.v[0], center.v[1], center.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "moving sphere" );

    // sphere now defines the interior sphere volume, corresponding to mcnp's notion of negative sense.
    // If positive sense is required, we must return the outside of the surface.

    if(positive){

	iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);
	iBase_EntityHandle volume;

	iGeom_subtractEnts( igm, universe_sphere, sphere, &volume, &igm_result);
	CHECK_IGEOM( igm_result, "subtracting sphere" );
	
	sphere = volume;

    }
    
    return sphere; 
  }

};

AbstractSurface& SurfaceCard::getSurface() {
  // SurfaceCard variables: surface, mnemonic, args, coord_xform
  
  if(!this->surface){
    if( mnemonic == "so"){
      this->surface = new SphereSurface( origin, args.at(0) );
    }
    else if( mnemonic == "sx"){
      this->surface = new SphereSurface( Vector3d( args.at(0), 0, 0 ), args.at(1) );
    }
    else if( mnemonic == "sy"){
      this->surface = new SphereSurface( Vector3d( 0, args.at(0), 0 ), args.at(1) );
    }
    else if( mnemonic == "sz"){
      this->surface = new SphereSurface( Vector3d( 0, 0, args.at(0) ), args.at(1) );
    }
    else if( mnemonic == "s" || mnemonic == "sph" ){
      this->surface = new SphereSurface( Vector3d( args ), args.at(3) );
    }
    else if( mnemonic == "p"){
      if( args.size() > 4 ){
	std::cerr << "Warning: surface of type P with more than 4 parameters." << std::endl;
	std::cerr << "         This surface type is unsupported; will proceed as if only 4 parameters given,\n";
	std::cerr << "         but the result will probably be incorrect!" << std::endl;
      }
      this->surface = new PlaneSurface( Vector3d( args ), args.at(3) );
    }
    else if( mnemonic == "px"){
      this->surface = new PlaneSurface( Vector3d( 1, 0, 0), args.at(0) );
    }
    else if( mnemonic == "py"){
      this->surface = new PlaneSurface( Vector3d( 0, 1, 0), args.at(0) );
    }
    else if( mnemonic == "pz"){
      this->surface = new PlaneSurface( Vector3d( 0, 0, 1), args.at(0) );
    }
    else if( mnemonic == "cx" ){
      this->surface = new CylinderSurface( CylinderSurface::X, args.at(0) );
    }
    else if( mnemonic == "cy" ){
      this->surface = new CylinderSurface( CylinderSurface::Y, args.at(0) );
    }
    else if( mnemonic == "cz" ){
      this->surface = new CylinderSurface( CylinderSurface::Z, args.at(0) );
    }
    else if( mnemonic == "c/x"){
      this->surface = new CylinderSurface( CylinderSurface::X, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/y"){
      this->surface = new CylinderSurface( CylinderSurface::Y, args.at(2), args.at(0), args.at(1) );
    }
    else if( mnemonic == "c/z"){
      this->surface = new CylinderSurface( CylinderSurface::Z, args.at(2), args.at(0), args.at(1) );
    }
    else{
      throw std::runtime_error( mnemonic + " is not a supported surface" );
    }

    if( this->coord_xform->hasData() ){
      const Transform& transform = coord_xform->getData();
      this->surface->setTransform( &transform );
    }
  }

  

  return *(this->surface);
}


//iBase_EntityHandle CellCard::define( iGeom_Instance& igm,  double universe_size){
iBase_EntityHandle defineCell( iGeom_Instance& igm, CellCard& cell, double universe_size ){

  int ident = cell.getIdent();
  const CellCard::geom_list_t& geom = cell.getGeom();
  InputDeck& deck = cell.getDeck();

  std::cerr << "Defining cell " << ident << std::endl;
  int igm_result;

  std::vector<iBase_EntityHandle> stack;
  for(CellCard::geom_list_t::const_iterator i = geom.begin(); i!=geom.end(); ++i){
    
    const CellCard::geom_list_entry_t& token = (*i);
    switch(token.first){
    case CellCard::CELLNUM:
      stack.push_back( defineCell( igm, *(deck.lookup_cell_card(token.second)), universe_size) );
      break;
    case CellCard::SURFNUM:
      {      
	int surface = token.second;
	bool pos = true;
	if( surface < 0){
	  pos = false; surface = -surface;
	}
	try{
	  AbstractSurface& surf = deck.lookup_surface_card( surface )->getSurface();
	  iBase_EntityHandle surf_handle = surf.define( pos, igm, universe_size );
	  stack.push_back(surf_handle);
	}
	catch(std::runtime_error& e) { std::cerr << e.what() << std::endl; }
      }
      break;
    case CellCard::INTERSECT:
      {
	assert( stack.size() >= 2 );
	iBase_EntityHandle s1 = stack.back(); stack.pop_back();
	iBase_EntityHandle s2 = stack.back(); stack.pop_back();
	iBase_EntityHandle result;
	iGeom_intersectEnts( igm, s1, s2, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Intersecting two entities" );
	stack.push_back(result);
      }
      break;
    case CellCard::UNION:
      {	
	assert( stack.size() >= 2 );
	iBase_EntityHandle s[2];
	s[0] = stack.back(); stack.pop_back();
	s[1] = stack.back(); stack.pop_back();
	iBase_EntityHandle result;
	iGeom_uniteEnts( igm, s, 2, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Uniting two entities" );
	stack.push_back(result);
      }
      break;
    case CellCard::COMPLEMENT:
      {
	assert (stack.size() >= 1 );
      	iBase_EntityHandle universe_sphere = makeUniverseSphere(igm, universe_size);
	iBase_EntityHandle s = stack.back(); stack.pop_back();
	iBase_EntityHandle result;

	iGeom_subtractEnts( igm, universe_sphere, s, &result, &igm_result);
	CHECK_IGEOM( igm_result, "Complementing an entity" );
	stack.push_back(result);
      }
      break;
    default:
      throw std::runtime_error( "Unexpected token while evaluating cell geometry");
      break;
    }
  }

  assert( stack.size() == 1);

  iBase_EntityHandle cellHandle = stack[0];

  if( cell.getTrcl().hasData() ){
    cellHandle = applyTransform( cell.getTrcl().getData(), igm, cellHandle );
  }

  return cellHandle;

}



void InputDeck::createGeometry(){

  iGeom_Instance igm;
  int igm_result;
  
  iGeom_newGeom( 0, &igm, &igm_result, 0);
  CHECK_IGEOM( igm_result, "Initializing iGeom");

  double universe_size = 0;
  for( surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
    universe_size = std::max( universe_size, (*i)->getSurface().getFarthestExtentFromOrigin() );
    } catch(std::runtime_error& e){}
  }
  double translation_addition = 0;
  for( data_card_list::iterator i = datacards.begin(); i!=datacards.end(); ++i){
    DataCard* c = *i;
    if( c->getKind() == DataCard::TR ){
      double tform_len = dynamic_cast<DataRef<Transform>*>(c)->getData().getTranslation().length();
      translation_addition = std::max (translation_addition, tform_len );
    }
  }
  universe_size += translation_addition;
  universe_size *= 1.2;

  std::cout << "Universe size: " << universe_size << " (trs added " << translation_addition << ")" << std::endl;
	
  size_t num_cells = cells.size();
  iBase_EntityHandle *cell_array = new iBase_EntityHandle[ num_cells ];
  int count = 0;

  for( cell_card_list::iterator i = cells.begin(); i!=cells.end(); ++i){
    cell_array[count] = defineCell( igm, *(*i), universe_size );
    count++;
  }

  iGeom_imprintEnts( igm, cell_array, num_cells, &igm_result );
  CHECK_IGEOM( igm_result, "Imprinting all cells" );

  //double tolerance = universe_size / 1.0e6;
  double tolerance = .001;
  std::cout << "Tolerance: " << tolerance << std::endl;
  iGeom_mergeEnts( igm, cell_array, num_cells,  tolerance, &igm_result );
  CHECK_IGEOM( igm_result, "Merging all cells" );

  std::string outName = "out.sat";
  iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
  CHECK_IGEOM( igm_result, "saving the output file "+outName );

}

  
int main(int argc, char* argv[]){

  std::string input_file = "INP";
  if(argc > 1){ input_file = argv[1]; }

  std::ifstream input(input_file.c_str(), std::ios::in );
    InputDeck& d = InputDeck::build(input);

    InputDeck::surface_card_list& surfaces = d.getSurfaces();
    for( InputDeck::surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
      try{
	const AbstractSurface& s = (*i)->getSurface();
	std::cout << "Distance from origin: " << s.getFarthestExtentFromOrigin() << std::endl;
      }
      catch(std::runtime_error& e){
	std::cout << e.what() << std::endl;
      }
    }

    d.createGeometry();

    return 0;
    

    int igm_result;
    iGeom_Instance igm;
    iGeom_newGeom( 0, &igm, &igm_result, 0);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error initializing iGeom" << std::endl;
    }

    /**\brief Create a sphere centered on the origin
     */
    iBase_EntityHandle sphere;
    iGeom_createSphere( igm, 150, &sphere, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error making sphere" << std::endl;
    }

    std::cout << "s1" << std::endl;

    iBase_EntityHandle sphere2;
    iGeom_createSphere( igm, 35, &sphere2, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error making sphere2" << std::endl;
    }

    std::cout << "s2" << std::endl;

    iGeom_moveEnt( igm, &sphere2, 150, 0, 0, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error moving sphere2" << std::endl;
    }

    std::cout << "move" << std::endl;

    iBase_EntityHandle intersection;
    iGeom_subtractEnts( igm, sphere, sphere2, &intersection, &igm_result);
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error doing intersection" << std::endl;
    }
    
    std::cout << "intersect" << std::endl;

    iBase_EntityHandle final;
    iGeom_sectionEnt( igm, &intersection, 0, 1, 0, 0, false, &final, &igm_result);

    std::string outName = "out.sat";
    iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
    if(igm_result != iBase_SUCCESS){
      std::cerr << "Error saving iGeom" << std::endl;
    }

}
