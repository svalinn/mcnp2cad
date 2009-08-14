#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <set>
#include <map>
#include <cmath>
#include <sstream>

#include <cassert>

#include "iGeom.h"
#include "geometry.hpp"
#include "MCNPInput.hpp"
#include "options.hpp"


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


typedef std::vector<iBase_EntityHandle> entity_collection_t;

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
    iGeom_rotateEnt( igm, e, t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
    CHECK_IGEOM( igm_result, "applying rotation" );
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

  if( rev_t.hasRot() ){
    const Vector3d& axis = rev_t.getAxis();
    iGeom_rotateEnt( igm, e, rev_t.getTheta(), axis.v[0], axis.v[1], axis.v[2], &igm_result );
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
    iGeom_sectionEnt( igm, world_sphere, 
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






class GeometryContext {

protected:
  iGeom_Instance& igm;
  InputDeck& deck;
  double world_size;
  int universe_depth;

  typedef std::pair<int, double> material_t;
  static material_t make_mat( int i, double d ){ return std::make_pair(i,d); }

  std::set< material_t > material_ids;
  std::map<iBase_EntityHandle, material_t > material_map;

public:
  GeometryContext( iGeom_Instance& igm_p, InputDeck& deck_p ) :
    igm(igm_p), deck(deck_p), world_size(0.0), universe_depth(0)
  {}

  bool defineLatticeNode( const Lattice& lattice, int lattice_universe, iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell,
			  int x, int y, int z, entity_collection_t& accum );
  

  entity_collection_t defineCell( CellCard& cell, bool defineEmbedded, iBase_EntityHandle lattice_shell );
  entity_collection_t populateCell( CellCard& cell, iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell );
 

  entity_collection_t defineUniverse( int universe, iBase_EntityHandle container, const Transform* transform );
  

  void setMaterial( iBase_EntityHandle cell, int material, double density );
  void updateMaps ( iBase_EntityHandle old_cell, iBase_EntityHandle new_cell );

  bool mapSanityCheck( iBase_EntityHandle* cells, size_t count );
  void setMaterialsAsGroups( );

  void createGeometry( );

};

void GeometryContext::setMaterial( iBase_EntityHandle cell, int material, double density ){

  
  material_t mat = make_mat( material, density );
  material_map.insert( std::make_pair( cell, mat ) );
  material_ids.insert( mat );

  if( OPT_DEBUG ){ std::cout << "Updated cell with new material, num materials now " << material_ids.size() << std::endl; }
}

void GeometryContext::updateMaps( iBase_EntityHandle old_cell, iBase_EntityHandle new_cell ){

 
  if( material_map.find( old_cell ) != material_map.end() ){
    if( new_cell != NULL ){
      material_map[new_cell] = material_map[old_cell];
    }
    material_map.erase( old_cell);
  }
}

void GeometryContext::setMaterialsAsGroups( ){
  int igm_result;
  
  std::map< material_t, iBase_EntitySetHandle> groups;

  std::string name_tag_id = "NAME";
  int name_tag_maxlength = 64;
  iBase_TagHandle name_tag;
  iBase_EntitySetHandle last_set;

  iGeom_getTagHandle( igm, name_tag_id.c_str(), &name_tag, &igm_result, name_tag_id.length() );
  CHECK_IGEOM( igm_result, "Looking up NAME tag" );
  
  iGeom_getTagSizeBytes( igm, name_tag, &name_tag_maxlength, &igm_result );
  CHECK_IGEOM( igm_result, "Querying NAME tag length" );
  std::cout << "Name tag length: " << name_tag_maxlength << " actual id " << name_tag << std::endl;

  for( std::map<iBase_EntityHandle,material_t>::iterator i = material_map.begin(); i != material_map.end(); ++i ){
    std::map< material_t, iBase_EntitySetHandle>::iterator j = groups.find((*i).second);

    if( j == groups.end() ){ 
      // create a new named entity set corresponding to the material *i.second, and name it
      iBase_EntitySetHandle set;
      iGeom_createEntSet( igm, 0, &set, &igm_result );
      CHECK_IGEOM( igm_result, "Creating a new entity set " );
      std::cout << "New set " << set << std::endl;

      last_set = set;

      groups[(*i).second] = set;
      j = groups.find((*i).second); 
      assert( j != groups.end() );
    }

    //add *i.first (an entity) to *j.second (an entity set) 
    iGeom_addEntToSet( igm, (*i).first, (*j).second, &igm_result );
    CHECK_IGEOM( igm_result, "Adding entity to material set" );

    //iGeom_addPrntChld( igm, &((*j).second), (void**)&((*i).first), &igm_result );
    //CHECK_IGEOM( igm_result, "Adding entity as child" );
    if( OPT_DEBUG ){ std::cout << "Added " << (*i).first << " to set at " << (*j).second << std::endl; }

  }

  for(  std::map< material_t, iBase_EntitySetHandle>::iterator i = groups.begin(); i != groups.end(); ++i){
    
    std::string name;
      std::stringstream formatter;
      formatter << "mat_" << (*i).first.first << "_rho_" << (*i).first.second;
      formatter >> name;
      
      if( name.length() > static_cast<unsigned>(name_tag_maxlength) ){
	name.resize( name_tag_maxlength -1 );
      }

      if( OPT_DEBUG ){ std::cout << "Creating material group " << name << " (" << (*i).second << ")" << std::endl; }

      iGeom_setEntSetData( igm, (*i).second, name_tag, name.c_str(), name.length(), &igm_result );
      CHECK_IGEOM( igm_result, "Naming an entity set" );

  }
  return;

  iBase_TagHandle th[12];
  int twelve = 12;
  int th_size;
  iBase_TagHandle *thp = &th[0];
  iGeom_getAllEntSetTags( igm, last_set, &thp, &twelve, &th_size, &igm_result );
  CHECK_IGEOM( igm_result, "Hack" );
  std::cout << "last_set has " << th_size << "tags" << std::endl;
  for( int i = 0 ; i < th_size; ++i ){
    //char str[32][32];
    //char* strp = &str[0];
    //std::string str;
    //std::string * str_p = &str;
    char str[32];
    int strlength = 32;
    int strsize;
    iGeom_getEntSetData( igm, last_set, name_tag, (char**)(&(str)), &strlength, &strsize, &igm_result );
    std::cout << str << std::endl;
  }
}

bool GeometryContext::mapSanityCheck( iBase_EntityHandle* cells, size_t count){ 
  bool good = true;
  int igm_result;

  iBase_EntitySetHandle rootset;
  iGeom_getRootSet( igm, &rootset, &igm_result );
  CHECK_IGEOM( igm_result, "Getting root set for sanity check" );

  int num_regions;
  iGeom_getNumOfType( igm, rootset, iBase_REGION, &num_regions, &igm_result );
  CHECK_IGEOM( igm_result, "Getting num regions for sanity check" );

  iBase_EntityHandle * handle_vector = new iBase_EntityHandle[ num_regions ];
  int size = 0;

  std::cout << "Map sanity check: num_regions = " << num_regions << std::endl;
  iGeom_getEntities( igm, rootset, iBase_REGION, &handle_vector, &num_regions, &size, &igm_result );
  CHECK_IGEOM( igm_result, "Getting entities for sanity check" );

  std::cout << "Map sanity check: root set size = " << size << " (" << num_regions << ")" << std::endl;
  std::cout << "Cell count: " << count << std::endl;
  if( static_cast<unsigned>(size) != count ){ std::cout << "WARNING: sizes differ; there may be gremlins in your geometry." << std::endl; }

  std::cout << "Map sanity check: cells with material properties set = " << material_map.size() << std::endl;

  // sanity conditions: all the keys in material_map are in the cells list
  std::set< iBase_EntityHandle > allRegions;
  for( size_t i = 0; i < count; ++i ){
    allRegions.insert( cells[i] );
  }
  
  for( std::map<iBase_EntityHandle,material_t>::iterator i = material_map.begin(); i!=material_map.end(); ++i){
    std::pair<iBase_EntityHandle,material_t> kv = *i;
    bool check = allRegions.find( kv.first ) != allRegions.end();
    if( !check ){
      std::cout << kv.first << " is not in all regions!" << std::endl;
    }
    good = good && check;
  }

  if( good ){ std::cout << "Map sanity check: pass!"  << std::endl; }
  else{ std::cout << "WARNING: Failed map sanity check!" << std::endl; }

  return good;
}


bool GeometryContext::defineLatticeNode(  const Lattice& lattice, int lattice_universe,
					  iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell,
					  int x, int y, int z, entity_collection_t& accum )
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
    if( OPT_DEBUG ) std::cout << " node failed bbox check" << std::endl;
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
    node_subcells = defineUniverse(  fn->getFillingUniverse(), cell_copy_unmoved, (fn->hasTransform() ? &(fn->getTransform()) : NULL ) );
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
      if( OPT_DEBUG ) std::cout << " node defined successfully" << std::endl;
      accum.push_back( result );
      success = true;
    }
    else{ 
      // lattice_shell_copy and node_subcells[i] were deleted by intersectIfPossible(),
      // so there's no need to delete them explicitly
      if( OPT_DEBUG ) std::cout << " node failed intersection" << std::endl;
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

typedef struct{ int v[3]; } int_triple;

static std::vector<int_triple> makeGridShellOfRadius( int r, int dimensions ){
  if( r == 0 ){ 
    int_triple v; v.v[0] = v.v[1] = v.v[2] = 0;
    return std::vector<int_triple>(1,v);
  }
  else{
    std::vector<int_triple> ret;
    
    int jmin = dimensions > 1 ? -r : 0;
    int jmax = dimensions > 1 ?  r : 0;
    int kmin = dimensions > 2 ? -r : 0;
    int kmax = dimensions > 2 ?  r : 0;
    for( int i = -r; i <= r; ++i ){
      for( int j = jmin;j <= jmax; ++j ){
	for( int k = kmin; k <= kmax; ++k ){
	  if( i == -r || i == r ||
	      j == -r || j == r ||
	      k == -r || k == r ){
	    int_triple v;
	    v.v[0] = i; 
	    v.v[1] = j;
	    v.v[2] = k;
	    ret.push_back(v);
	  }
	}
      }
    }
    return ret;
  }
}

entity_collection_t GeometryContext::populateCell( CellCard& cell,  iBase_EntityHandle cell_shell, 
						   iBase_EntityHandle lattice_shell = NULL )
{
  
  if( OPT_DEBUG ) std::cout << "Populating cell " << cell.getIdent() << std::endl;


  if( !cell.hasFill() && !cell.isLattice() ){
    // nothing inside this cell
    if( cell.getMat() != 0 ){ setMaterial( cell_shell, cell.getMat(), cell.getRho() ); }
    return entity_collection_t(1, cell_shell );
  }
  else if(cell.hasFill() && !cell.isLattice()){
    // define a simple (non-lattice) fill
    
    const FillNode& n = cell.getFill().getOriginNode();
    int filling_universe = n.getFillingUniverse();
    
    if( OPT_DEBUG ){
      std::cout << "Creating cell " << cell.getIdent() 
		<< ", which is filled with universe " << filling_universe << std::endl;
    }
    
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

    entity_collection_t subcells = defineUniverse(  filling_universe, cell_shell, t );
 
    return subcells;
     
  }
  else {
    // cell is a lattice, bounded by lattice_shell.  cell_shell is the origin element of the lattice and
    // cell->getLattice() has the lattice parameters.

    assert(lattice_shell);
    
    if( OPT_VERBOSE ) std::cout << "Creating cell " << cell.getIdent() << "'s lattice" << std::endl;

    entity_collection_t subcells;
        
    const Lattice& lattice = cell.getLattice();
    int num_dims = lattice.numFiniteDirections();
    
    if( OPT_DEBUG ) std::cout << "  lattice num dims: " << num_dims << std::endl;

    if( lattice.isFixedSize() ){

      if( OPT_DEBUG ) std::cout << "Defining fixed lattice" << std::endl;

      irange xrange = lattice.getXRange(), yrange = lattice.getYRange(), zrange = lattice.getZRange();

      for( int i = xrange.first; i <= xrange.second; ++i){
	for( int j = yrange.first; j <= yrange.second; ++j ){
	  for( int k = zrange.first; k <= zrange.second; ++k ){

	    if( OPT_DEBUG ) std::cout << "Defining lattice node " << i << ", " << j << ", " << k << std::endl;

	    /* bool success = */ defineLatticeNode( lattice, cell.getUniverse(), cell_shell, lattice_shell, i, j, k, subcells );

	    if( num_dims < 3 ) break; // from z loop
	  }
	  if( num_dims < 2 ) break; // from y loop
	}
      }

    }
    else{

      if( OPT_DEBUG ) std::cout << "Defining infinite lattice" << std::endl;

      bool done = false;
      int radius = 0;

      while( !done ){
	
	done = true;
	std::vector<int_triple> shell = makeGridShellOfRadius(radius++, num_dims);

	for( std::vector<int_triple>::iterator i = shell.begin(); i!=shell.end(); ++i){
	  int x = (*i).v[0];
	  int y = (*i).v[1];
	  int z = (*i).v[2];
	  
	  if( OPT_DEBUG ) std::cout << "Defining lattice node " << x << ", " << y << ", " << z << std::endl;

	  bool success = defineLatticeNode( lattice, cell.getUniverse(), cell_shell, lattice_shell, x, y, z, subcells );
	  if( success ) done = false;

	}	
      }
    }

    int igm_result;
    iGeom_deleteEnt( igm, cell_shell, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting cell shell after building lattice" );
    iGeom_deleteEnt( igm, lattice_shell, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting lattice shell after building lattice" );

    return subcells;
  }
}

entity_collection_t GeometryContext::defineCell(  CellCard& cell,  bool defineEmbedded = true, 
						  iBase_EntityHandle lattice_shell = NULL )
{
  int ident = cell.getIdent();
  const CellCard::geom_list_t& geom = cell.getGeom();
 
  if( OPT_VERBOSE ) std::cout << "Defining cell " << ident << std::endl;

  int igm_result;

  entity_collection_t tmp;

  std::vector<iBase_EntityHandle> stack;
  for(CellCard::geom_list_t::const_iterator i = geom.begin(); i!=geom.end(); ++i){
    
    const CellCard::geom_list_entry_t& token = (*i);
    switch(token.first){
    case CellCard::CELLNUM:
      // a cell number appears in a geometry list only because it is being complemented with the # operator
      // thus, when defineCell is called on it, set defineEmbedded to false
      tmp = defineCell( *(deck.lookup_cell_card(token.second)), false);
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
    return populateCell( cell, cellHandle, lattice_shell );
  }
  else{
    return entity_collection_t( 1, cellHandle );
  }
  
}

entity_collection_t GeometryContext::defineUniverse( int universe, iBase_EntityHandle container = NULL, 
						     const Transform* transform = NULL )
{

  if( OPT_VERBOSE ) std::cout << "Defining universe " << universe << std::endl;

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
    entity_collection_t tmp = defineCell( *(*i), true, lattice_shell );
    for( size_t i = 0; i < tmp.size(); ++i){
      subcells.push_back( tmp[i] );
    }
  }
  
  if( transform ){
    for( size_t i = 0; i < subcells.size(); ++i){
      subcells[i] = applyTransform( *transform, igm, subcells[i] );      
    }
  }

  if( container && !lattice_shell ){
    
    int igm_result;
    
    for( size_t i = 0; i < subcells.size(); ++i ){
      
      if( boundBoxesIntersect( igm, subcells[i], container )){
	iBase_EntityHandle container_copy;
	iGeom_copyEnt( igm, container, &container_copy, &igm_result);
	CHECK_IGEOM( igm_result, "Copying a universe-bounding cell" );
	
	iBase_EntityHandle subcell_bounded;
	
	if( OPT_DEBUG) std::cout << "Bounding a universe cell..." << std::endl;
	bool valid_result = intersectIfPossible( igm, container_copy, subcells[i], &subcell_bounded );
	if( valid_result ){
	  updateMaps( subcells[i], subcell_bounded );
	  subcells[i] = subcell_bounded;
	}
	else{
	  updateMaps( subcells[i], NULL );
	  subcells.erase( subcells.begin()+i );
	  i--;
	}

      }
      
    }
	
    iGeom_deleteEnt( igm, container, &igm_result );
    CHECK_IGEOM( igm_result, "Deleting a bounding cell" );
  }

  if( OPT_VERBOSE ) std::cout << "Done defining universe " << universe << std::endl;
  return subcells;
 
}

void GeometryContext::createGeometry( ){

  int igm_result;
 
  InputDeck::surface_card_list surfaces  = deck.getSurfaces();
  InputDeck::data_card_list    datacards = deck.getDataCards();

  for( InputDeck::surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
    world_size = std::max( world_size, (*i)->getSurface().getFarthestExtentFromOrigin() );
    } catch(std::runtime_error& e){}
  }
  double translation_addition = 0;
  for( InputDeck::data_card_list::iterator i = datacards.begin(); i!=datacards.end(); ++i){
    DataCard* c = *i;
    if( c->getKind() == DataCard::TR ){
      double tform_len = dynamic_cast<DataRef<Transform>*>(c)->getData().getTranslation().length();
      translation_addition = std::max (translation_addition, tform_len );
    }
  }
  world_size += translation_addition;
  world_size *= 1.2;

  std::cout << "World size: " << world_size << " (trs added " << translation_addition << ")" << std::endl;

  std::cout << "Defining geometry..." << std::endl;

  entity_collection_t defined_cells = defineUniverse( 0 );

  size_t count = defined_cells.size();
  iBase_EntityHandle *cell_array = new iBase_EntityHandle[ count ];
  for( unsigned int i = 0; i < count; ++i ){
    cell_array[i] = defined_cells[i];
  }


  if( OPT_DEBUG ){ mapSanityCheck(cell_array, count); } 
  setMaterialsAsGroups();

  std::cout << "Imprinting all...\t\t\t" << std::flush;
  iGeom_imprintEnts( igm, cell_array, count, &igm_result );
  CHECK_IGEOM( igm_result, "Imprinting all cells" );
  std::cout << " done." << std::endl;

  double tolerance = world_size / 1.0e7;
  //double tolerance = .001;
  std::cout << "Merging, tolerance=" << tolerance << "...\t\t" << std::flush;
  iGeom_mergeEnts( igm, cell_array, count,  tolerance, &igm_result );
  CHECK_IGEOM( igm_result, "Merging all cells" );
  std::cout << " done." << std::endl;

  std::string outName = opt.output_file;
  std::cout << "Saving file \"" << outName << "\"...\t\t\t" << std::flush;
  iGeom_save( igm, outName.c_str(), "", &igm_result, outName.length(), 0 );
  CHECK_IGEOM( igm_result, "saving the output file "+outName );
  std::cout << " done." << std::endl;

}

void debugSurfaceDistances( InputDeck& deck, std::ostream& out = std::cout ){

  InputDeck::surface_card_list& surfaces = deck.getSurfaces();
  for( InputDeck::surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
      const AbstractSurface& s = (*i)->getSurface();
      out << "S" << (*i)->getIdent() << " distance from origin: " << s.getFarthestExtentFromOrigin() << std::endl;
    }
    catch(std::runtime_error& e){
      std::cerr << "Error debugging surface distances: " << e.what() << std::endl;
      throw;
    }
  }
  
}

void printHelp( const char* progname, std::ostream& out );

struct program_option_struct opt;

int main(int argc, char* argv[]){


  if( argc < 2 ){
    printHelp( argv[0], std::cout );
    return 0;
  }


#ifdef USING_CUBIT
  CubitMessage::instance()->set_info_flag( false );
#endif

  opt.input_file = NULL;
  opt.output_file = "out.sat";
  opt.igeom_init_options = "";
  

  bool DiFlag = false, DoFlag = false;
  
  for( int i = 1; i < argc; ++i){
    std::string arg = argv[i];
    if( arg == "-h" || arg == "--help" ){
      printHelp( argv[0], std::cout );
      return 0;
    }
    else if( arg == "-o" ){
      if( i+1 >= argc ){
	std::cerr << "Error: -o option requires an argument" << std::endl;
	printHelp( argv[0], std::cerr );
	return 1;
      }
      else{
	opt.output_file = argv[i+1];
	i++;
      }
    }
    else if( arg == "-v" ){
      opt.verbose = true;
    }
    else if( arg == "-D" ){
      opt.debug   = true;
    }
    else if( arg == "-Do" ){
      DoFlag = true;
    }
    else if (arg == "-Di" ){
      DiFlag = true;
    }
#ifdef USING_CUBIT
    else if (arg == "-Cv" ){
      CubitMessage::instance()->set_info_flag( true );
    }
#endif
    else if ( arg[0] == '-'){ 
      std::cerr << "Error: unknown option  \"" << arg << "\"" << std::endl;
      printHelp( argv[0], std::cerr );
      return 1;
    }
    else{
      opt.input_file = argv[i];
    }

  }

  if( opt.input_file == NULL ){
    std::cerr << "Error: no input file given" << std::endl;
    printHelp( argv[0], std::cerr );
    return 1;
  }
  
  std::ifstream input(opt.input_file, std::ios::in );
  if( !input.is_open() ){
    std::cerr << "Error: couldn't open file \"" << opt.input_file << "\"" << std::endl;
    return 1;
  }
  
  std::cout << "Reading input file..." << std::endl;

  // if -Di and not -D, set debugging to be true for InputDeck::build() call only
  if( DiFlag && !OPT_DEBUG){
    opt.debug = true;
  }
  else{ DiFlag = false; }

  InputDeck& deck = InputDeck::build(input);
  std::cout << "Done reading input." << std::endl;

  // turn off debug if it was set by -Di only
  if( DiFlag ){ opt.debug = false; }
  
  if( DoFlag && !OPT_DEBUG ){ opt.debug = true; }

  if( OPT_DEBUG ){ 
    debugSurfaceDistances( deck );
  }

  iGeom_Instance igm;
  int igm_result; 

  iGeom_newGeom( opt.igeom_init_options, &igm, &igm_result, std::strlen(opt.igeom_init_options) );
  CHECK_IGEOM( igm_result, "Initializing iGeom");

  GeometryContext context( igm, deck );
  context.createGeometry();
  
  return 0;
    
}

void printHelp( const char* progname, std::ostream& out ){
  
  out << "usage: " << progname << " [options] input_file\n" << std::endl;
  out << "options:\n";
  out << 
    "  -h, --help            Show this message and exit\n" <<
    "  -o OUTPUT             Give name of output file (default: " << OPT_DEFAULT_OUTPUT_FILENAME << ")\n" <<
    "  -v                    Verbose output\n" <<
    "  -D                    Debugging (super-verbose) output\n" <<
    "  -Di                   Debugging output for MCNP parsing phase only\n" <<
    "  -Do                   Debugging output for iGeom output phase only\n" << 
#ifdef USING_CUBIT
    "  -Cv                   Verbose messages from CGM library\n" <<
#endif
    std::endl;

}
