#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <cmath>

#include <cassert>

//#include "MCNPInputDeck.hpp"
#include "iGeom.h"
#include "geometry.hpp"
#include "MCNPInput.hpp"


#ifdef USING_CUBIT
#include <CubitMessage.hpp>
#endif


#define MY_BUF_SIZE 512
static char m_buf[MY_BUF_SIZE];

#define CHECK_IGEOM(err, msg) \
  do{/*std::cout << msg << std::endl;*/ if((err) != iBase_SUCCESS){	\
    std::cerr << "iGeom error (" << err << "): " << msg << std::endl;	\
    iGeom_getDescription( igm, m_buf, &err, MY_BUF_SIZE); \
    std::cerr << " * " << m_buf << std::endl; \
     } } while(0) 

static bool intersectIfPossible( iGeom_Instance igm, 
				 iBase_EntityHandle h1, iBase_EntityHandle h2, iBase_EntityHandle* result, 
				 bool delete_on_failure = true)
{
  int igm_result;
  iGeom_intersectEnts( igm, h1, h2, result, &igm_result);
 
  if( igm_result == iBase_SUCCESS ){
    return true;
  }
  else{
    if( delete_on_failure ){
      iGeom_deleteEnt( igm, h1, &igm_result);
      CHECK_IGEOM(igm_result, "deleting an intersection candidate");
      iGeom_deleteEnt( igm, h2, &igm_result);
      CHECK_IGEOM(igm_result, "deleting an intersection candidate");
    }
    return false;
  }
}

static bool boundBoxesIntersect( iGeom_Instance igm, iBase_EntityHandle h1, iBase_EntityHandle h2 ){

  Vector3d h1_min, h1_max, h2_min, h2_max;
  int igm_result;
  
  iGeom_getEntBoundBox( igm, h1, h1_min.v, h1_min.v+1, h1_min.v+2, h1_max.v, h1_max.v+1, h1_max.v+2, &igm_result );
  CHECK_IGEOM( igm_result, "Getting bounding box h1" );
  iGeom_getEntBoundBox( igm, h2, h2_min.v, h2_min.v+1, h2_min.v+2, h2_max.v, h2_max.v+1, h2_max.v+2, &igm_result );
  CHECK_IGEOM( igm_result, "Getting bounding box h2" );

  bool ret = false;

  for( int i = 0; i < 3 && ret == false; ++i ){
    ret = ret || ( h1_min.v[i] > h2_max.v[i] );
    ret = ret || ( h2_min.v[i] > h1_max.v[i] );
  }

  return !ret;

}

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
  virtual iBase_EntityHandle define( bool positive, iGeom_Instance& igm, double world_size );

protected:
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ) = 0;
};


iBase_EntityHandle applyTransform( const Transform& t, iGeom_Instance& igm, iBase_EntityHandle& e ) {
  
  int igm_result;
  if( t.hasRot() ){
    const Vector3d& axis = t.getAxis();
    iGeom_rotateEnt( igm, &e, t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "applying rotation" );
  }
  
  const Vector3d& translation = t.getTranslation();
  iGeom_moveEnt( igm, &e, translation.v[0], translation.v[1], translation.v[2], &igm_result);
  CHECK_IGEOM( igm_result, "applying translation" );
  
  return e;
}

iBase_EntityHandle applyReverseTransform( const Transform& tx, iGeom_Instance& igm, iBase_EntityHandle& e ) {
  
  int igm_result;
  Transform rev_t = tx.reverse();

  const Vector3d& translation = rev_t.getTranslation();
  iGeom_moveEnt( igm, &e, translation.v[0], translation.v[1], translation.v[2], &igm_result);
  CHECK_IGEOM( igm_result, "applying reverse translation" );

  if( rev_t.hasRot() ){
    const Vector3d& axis = rev_t.getAxis();
    iGeom_rotateEnt( igm, &e, rev_t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "applying rotation" );
  }
  
  
  return e;
}



static Vector3d origin(0,0,0);


iBase_EntityHandle makeWorldSphere( iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle world_sphere;
  int igm_result;
  iGeom_createSphere( igm, world_size, &world_sphere, &igm_result);
  CHECK_IGEOM( igm_result, "making world sphere" );
  return world_sphere;
}

iBase_EntityHandle AbstractSurface::define( bool positive, iGeom_Instance& igm, double world_size ){
  iBase_EntityHandle handle = this->getHandle( positive, igm, world_size );
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
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size){

    int igm_result;
    iBase_EntityHandle world_sphere = makeWorldSphere(igm, world_size);
    iBase_EntityHandle hemisphere;
    // note the reversal of sense in this call; mcnp and igeom define it differently.
    iGeom_sectionEnt( igm, &world_sphere, 
		      normal.v[0], normal.v[1], normal.v[2], offset, !positive, &hemisphere, &igm_result);
    CHECK_IGEOM( igm_result, "Sectioning world for a plane" );
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
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){
    int igm_result;

    iBase_EntityHandle cylinder;
    iGeom_createCylinder( igm, 2.0 * world_size, radius, 0, &cylinder, &igm_result);
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

    iBase_EntityHandle world_sphere = makeWorldSphere( igm, world_size );
    iBase_EntityHandle final_cylinder;

    if( positive ){
      iGeom_subtractEnts( igm, world_sphere, cylinder, &final_cylinder, &igm_result);
      CHECK_IGEOM( igm_result, "making clipped cylinder" );
    }
    else{
      iGeom_intersectEnts( igm, world_sphere, cylinder, &final_cylinder, &igm_result);
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
  virtual iBase_EntityHandle getHandle( bool positive, iGeom_Instance& igm, double world_size ){

    int igm_result;
    iBase_EntityHandle sphere;

    iGeom_createSphere( igm, radius, &sphere, &igm_result);
    CHECK_IGEOM( igm_result, "making sphere" );

    iGeom_moveEnt( igm, &sphere, center.v[0], center.v[1], center.v[2], &igm_result );
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
      Vector3d v(args);
      this->surface = new PlaneSurface( v, args.at(3)/v.length() );
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

typedef std::vector<iBase_EntityHandle> entity_collection_t;

entity_collection_t defineUniverse( iGeom_Instance&, InputDeck&, int, double, iBase_EntityHandle, const Transform* );

bool defineLatticeNode( iGeom_Instance& igm, const Lattice& lattice, int lattice_universe,
			iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell,
			int x, int y, int z, entity_collection_t& accum,
			InputDeck& deck, double world_size )
{
  const FillNode* fn = &(lattice.getFillForNode( x, y, z ));				
  Transform t = lattice.getTxForNode( x, y, z );	
  int igm_result;
  
  iBase_EntityHandle cell_copy;
  iGeom_copyEnt( igm, cell_shell, &cell_copy, &igm_result );
  CHECK_IGEOM( igm_result, "Copying a lattice cell shell" );
  cell_copy = applyTransform( t, igm, cell_copy );
  
  if( !boundBoxesIntersect( igm, cell_copy, lattice_shell ) ){
    iGeom_deleteEnt( igm, cell_copy, &igm_result);
    CHECK_IGEOM( igm_result, "Deleting a lattice cell shell" );
    std::cout << "failed bbox check" << std::endl;
    return false;
  }

  entity_collection_t node_subcells;
  if( fn->getFillingUniverse() == 0 ){
    // this node of the lattice "exists" in sense that it's well defined, however,
    // it is defined to be empty.  Delete the shell and return true.
    iGeom_deleteEnt( igm, cell_copy, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting a universe-0 lattice cell" );
    return true;
  }
  else if( fn->getFillingUniverse() == lattice_universe ){
    node_subcells.push_back( cell_copy );
  }
  else{

    iBase_EntityHandle cell_copy_unmoved;
    iGeom_copyEnt( igm, cell_shell, &cell_copy_unmoved, &igm_result );
    CHECK_IGEOM( igm_result, "Re-copying a lattice cell shell" );
    node_subcells = defineUniverse( igm, deck, fn->getFillingUniverse(), world_size, cell_copy_unmoved, (fn->hasTransform() ? &(fn->getTransform()) : NULL ) );
    for( size_t i = 0; i < node_subcells.size(); ++i ){
      node_subcells[i] = applyTransform( t, igm, node_subcells[i] );
    }

    iGeom_deleteEnt( igm, cell_copy, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting lattice cell copy" );

  }

  bool success = false;
  for( size_t i = 0; i < node_subcells.size(); ++i ){
    iBase_EntityHandle lattice_shell_copy;
    iGeom_copyEnt( igm, lattice_shell, &lattice_shell_copy, &igm_result );

    iBase_EntityHandle result;
    if( intersectIfPossible( igm, lattice_shell_copy, node_subcells[i], &result, true ) ){
      std::cout << "success" << std::endl;
      accum.push_back( result );
      success = true;
    }
    else{ 
      // lattice_shell_copy and node_subcells[i] were deleted by intersectIfPossible()
      std::cout << "Failed intersection" << std::endl;
    }
  }

  return success;
}

int quickPow( int x, unsigned int exp ){
  int ret = x;
  for( unsigned int i = 1; i < exp; i++){
    ret *= x;
  }
  return ret;
}

entity_collection_t populateCell( iGeom_Instance& igm, CellCard& cell, double world_size, 
				  iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell = NULL ){

  InputDeck& deck = cell.getDeck();
  std::cout << "Populating cell " << cell.getIdent() << std::endl;


  if( !cell.hasFill() && !cell.isLattice() ){
    // nothing more to do: return this cell
    return entity_collection_t(1, cell_shell );
  }
  else if(cell.hasFill() && !cell.isLattice()){
    // define a simple (non-lattice) fill
    
    const FillNode& n = cell.getFill().getOriginNode();
    int filling_universe = n.getFillingUniverse();
    std::cout << "Creating cell " << cell.getIdent() 
	      << ", which is filled with universe " << filling_universe << std::endl;
    
    
    // the contained universe is transformed by the FillNode's transform, if any, or
    // else by the cell's TRCL value, if any.
    const Transform* t;
    if( n.hasTransform() ){
      t = &(n.getTransform());
    } else if( cell.getTrcl().hasData() ){
      t = &(cell.getTrcl().getData() );
    } else { 
      t = NULL; 
    }

    entity_collection_t subcells = defineUniverse( igm, deck, filling_universe, world_size, cell_shell, t );
 
    return subcells;
     
  }
  else {
    // cell is a lattice, bounded by lattice_shell.  cell_shell is the origin element of the lattice and
    // cell->getLattice() has the lattice parameters.

    assert(lattice_shell);
    std::cout << "Creating cell " << cell.getIdent() << " lattice within a shell" << std::endl;
    entity_collection_t subcells;
        
    const Lattice& lattice = cell.getLattice();
    int num_dims = lattice.numFiniteDirections();
    std::cout << "Num dims " << num_dims << std::endl;

    if( lattice.isFixedSize() ){

      std::cout << "Defining fixed lattice" << std::endl;
      irange xrange = lattice.getXRange(), yrange = lattice.getYRange(), zrange = lattice.getZRange();

      for( int i = xrange.first; i <= xrange.second; ++i){
	for( int j = yrange.first; j <= yrange.second; ++j ){
	  for( int k = zrange.first; k <= zrange.second; ++k ){

	    std::cout << "Defining lattice node " << i << ", " << j << ", " << k << std::endl;
	    /* bool success = */ defineLatticeNode( igm, lattice, cell.getUniverse(), cell_shell, lattice_shell, i, j, k, subcells, deck, world_size );

	    if( num_dims < 3 ) break; // from z loop
	  }
	  if( num_dims < 2 ) break; // from y loop
	}
      }

    }
    else{
      int x = 0, y = 0, z = 0;
      bool xdone = false, ydone = false, zdone = false;
      int failcount = 0;
      
      const int try_factor = 1;
      
      do{
	
	y = 0;
	ydone = false;
	do{
	  
	  z = 0;
	  zdone = false;
	  do{
	    
	    
	    std::cout << "Defining lattice node " << x << ", " << y << ", " << z << std::endl;
	    bool success = defineLatticeNode( igm, lattice, cell.getUniverse(), cell_shell, lattice_shell, x, y, z, subcells, deck, world_size );
	    if(success){ failcount = 0; }
	    else{ failcount++; }
	    
	    if( z > 0 ){ z = -z; }
	    else{ z = (-z) + 1; }
	    
	    if( failcount >= (2*try_factor) ){ zdone = true; }
	    
	  }while(!zdone && num_dims >= 3);
	  
	  if( y > 0 ){ y = -y; }
	  else{ y = (-y) + 1; }
	  
	  if( failcount >= quickPow(2*try_factor,2) ){ ydone = true; }
	  
	}while (!ydone && num_dims >= 2);
	
	if( x > 0){ x = -x; }
	else{ x = (-x) + 1; }
	
	if( failcount >= quickPow(2*try_factor, 3) ){ xdone = true; }
	
      }while( !xdone );
      

    }

    int igm_result;
    iGeom_deleteEnt( igm, cell_shell, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting cell shell after building lattice" );
    iGeom_deleteEnt( igm, lattice_shell, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting lattice shell after building lattice" );

    return subcells;
  }
}

//iBase_EntityHandle CellCard::define( iGeom_Instance& igm,  double world_size){
entity_collection_t defineCell( iGeom_Instance& igm, CellCard& cell, double world_size, 
				bool defineEmbedded = true, iBase_EntityHandle lattice_shell = NULL ){

  int ident = cell.getIdent();
  const CellCard::geom_list_t& geom = cell.getGeom();
  InputDeck& deck = cell.getDeck();

  std::cout << "Defining cell " << ident << std::endl;
  int igm_result;

  entity_collection_t tmp;

  std::vector<iBase_EntityHandle> stack;
  for(CellCard::geom_list_t::const_iterator i = geom.begin(); i!=geom.end(); ++i){
    
    const CellCard::geom_list_entry_t& token = (*i);
    switch(token.first){
    case CellCard::CELLNUM:
      // a cell number appears in a geometry list only because it is being complemented with the # operator
      // thus, when defineCell is called on it, set defineEmbedded to false
      tmp = defineCell( igm, *(deck.lookup_cell_card(token.second)), world_size, false);
      assert(tmp.size() == 1);
      stack.push_back( tmp.at(0) );
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
	  iBase_EntityHandle surf_handle = surf.define( pos, igm, world_size );
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
	if( intersectIfPossible( igm, s1, s2, &result ) ){
	  stack.push_back(result);
	}
	else{
	  std::cout << "FAILED INTERSECTION CELL #" << cell.getIdent() << std::endl;
	  throw std::runtime_error("Intersection failed");
	}
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
      	iBase_EntityHandle world_sphere = makeWorldSphere(igm, world_size);
	iBase_EntityHandle s = stack.back(); stack.pop_back();
	iBase_EntityHandle result;

	iGeom_subtractEnts( igm, world_sphere, s, &result, &igm_result);
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

  if( defineEmbedded ){
    return populateCell( igm, cell, world_size, cellHandle, lattice_shell );
  }
  else{
    return entity_collection_t( 1, cellHandle );
  }
  
}

entity_collection_t defineUniverse( iGeom_Instance &igm, InputDeck& deck, int universe, double world_size, 
				    iBase_EntityHandle container = NULL, const Transform* transform = NULL ){

  std::cout << "Defining universe " << universe << std::endl;
  InputDeck::cell_card_list u_cells = deck.getCellsOfUniverse( universe );
  entity_collection_t subcells;

  iBase_EntityHandle lattice_shell = NULL;
  if( u_cells.size() == 1 && u_cells[0]->isLattice() ){
    lattice_shell = container;
    if(transform){
      lattice_shell = applyReverseTransform( *transform, igm, lattice_shell );
    }
  }

  for( InputDeck::cell_card_list::iterator i = u_cells.begin(); i!=u_cells.end(); ++i){
    entity_collection_t tmp = defineCell( igm, *(*i), world_size, true, lattice_shell );
    for( size_t i = 0; i < tmp.size(); ++i){
      subcells.push_back( tmp[i] );
    }
  }
  
  if( transform ){
    for( size_t i = 0; i < subcells.size(); ++i){
      subcells[i] = applyTransform( *transform, igm, subcells[i] );      
    }
  }

  if( container && !lattice_shell){
    
    int igm_result;
    
    for( size_t i = 0; i < subcells.size(); ++i ){
      
      if( boundBoxesIntersect( igm, subcells[i], container )){
	iBase_EntityHandle container_copy;
	iGeom_copyEnt( igm, container, &container_copy, &igm_result);
	CHECK_IGEOM( igm_result, "Copying a universe-bounding cell" );
	
	iBase_EntityHandle subcell_bounded;
	
	std::cout << "Bounding" << std::endl;
	bool valid_result = intersectIfPossible( igm, container_copy, subcells[i], &subcell_bounded );
	if( valid_result ){
	  subcells[i] = subcell_bounded;
	}
	else{
	  subcells.erase( subcells.begin()+i );
	  i--;
	}

      }
      
    }
	
    iGeom_deleteEnt( igm, container, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting a bounding cell" );
  }
  return subcells;
 
}

void InputDeck::createGeometry(){

  iGeom_Instance igm;
  int igm_result;
  
  iGeom_newGeom( "-q", &igm, &igm_result, 0);
  CHECK_IGEOM( igm_result, "Initializing iGeom");

  double world_size = 0;
  for( surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
    world_size = std::max( world_size, (*i)->getSurface().getFarthestExtentFromOrigin() );
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
  world_size += translation_addition;
  world_size *= 1.2;

  std::cout << "World size: " << world_size << " (trs added " << translation_addition << ")" << std::endl;

  entity_collection_t defined_cells = defineUniverse( igm, *this, 0, world_size );

  size_t count = defined_cells.size();
  iBase_EntityHandle *cell_array = new iBase_EntityHandle[ count ];
  for( unsigned int i = 0; i < count; ++i ){
    cell_array[i] = defined_cells[i];
  }


  std::cout << "Imprinting all..." << std::endl;
  iGeom_imprintEnts( igm, cell_array, count, &igm_result );
  CHECK_IGEOM( igm_result, "Imprinting all cells" );

  double tolerance = world_size / 1.0e7;
  //double tolerance = .001;
  std::cout << "Merging, tolerance: " << tolerance << std::endl;
  iGeom_mergeEnts( igm, cell_array, count,  tolerance, &igm_result );
  CHECK_IGEOM( igm_result, "Merging all cells" );

  std::string outName = "out.sat";
  std::cout << "Saving file: " << outName << std::endl;
  iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
  CHECK_IGEOM( igm_result, "saving the output file "+outName );

}

  
int main(int argc, char* argv[]){

#ifdef USING_CUBIT
  //  std::cout << CubitMessage::instance()->get_info_flag() << std::endl;
  CubitMessage::instance()->set_info_flag( false );
#endif

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
    
}
