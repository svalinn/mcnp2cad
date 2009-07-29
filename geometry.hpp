#ifndef MCNP2IGEOM_GEOMETRY_H
#define MCNP2IGEOM_GEOMETRY_H

#include <vector>
#include <cmath>
#include <iosfwd>

#include "dataref.hpp"

class Vector3d{

public:

  double v[3];
  Vector3d(){
    v[2] = v[1] = v[0] = 0;
  }
  
  Vector3d( const double p[3] ){
    v[0] = p[0];
    v[1] = p[1];
    v[2] = p[2];
  }

  Vector3d( double x, double y, double z ){
    v[0] = x; 
    v[1] = y;
    v[2] = z;
  }

  Vector3d( const std::vector<double>& p ){
    v[0] = p.at(0);
    v[1] = p.at(1);
    v[2] = p.at(2);
  }

  double length() const{
    return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
  }

  Vector3d normalize() const {
    double length = this->length();
    return Vector3d( v[0]/length, v[1]/length, v[2]/length );
  }

  Vector3d operator-() const {
    return Vector3d(-v[0], -v[1], -v[2]);
  }

  Vector3d reverse() const { 
    return -(*this);
  }

  Vector3d scale( double d ) const {
    return Vector3d(v[0]*d, v[1]*d, v[2]*d);
  }

  Vector3d operator*( double d ) const {
    return scale(d);
  }

  Vector3d add( const Vector3d& alt ) const {
    return Vector3d( v[0]+alt.v[0], v[1]+alt.v[1], v[2]+alt.v[2] );
  }

  Vector3d operator+( const Vector3d& alt ) const {
    return add(alt);
  }

  double dot( const Vector3d& alt ) const {
    return v[0]*alt.v[0] + v[1]*alt.v[1] + v[2]*alt.v[2];
  }

  Vector3d cross( const Vector3d& alt ) const { 
    Vector3d c;
    c.v[0] = v[1]*alt.v[2] - v[2]*alt.v[1];
    c.v[1] = v[2]*alt.v[0] - v[0]*alt.v[2];
    c.v[2] = v[0]*alt.v[1] - v[1]*alt.v[0];
    return c;
  }

};

std::ostream& operator<<(std::ostream& str, const Vector3d& v );

class Transform{

protected:
  Vector3d translation;
  bool has_rot;
  double rot_x, rot_y, rot_z;
  double raw_matrix[9];
  bool degree_format;

  void set_rots_from_matrix( );

public:
  Transform():translation(),has_rot(false){}
  Transform( const Vector3d& v ):translation(v),has_rot(false){}
  Transform( const std::vector< double >& inputs, bool degree_format_p = false );
  const Vector3d& getTranslation() const { return translation; }
  bool hasRot() const{ return has_rot; }
  double getRotX() const { return rot_x; }
  double getRotY() const { return rot_y; }
  double getRotZ() const { return rot_z; }
  void print( std::ostream& str ) const;

  Transform reverse() const;

};

class FillNode {

protected:
  int universe;
  DataRef<Transform>* tr;

public:
  FillNode():
    universe(0), tr(new NullRef<Transform>())
  {}

  FillNode( int universe_p ):
    universe(universe_p), tr(new NullRef<Transform>())
  {}

  FillNode( int universe_p, DataRef<Transform>* tr_p ):
    universe(universe_p), tr(tr_p)
  {}

  FillNode( const FillNode& node_p ):
    universe(node_p.universe), tr(node_p.tr->clone())
  {}

  FillNode& operator=( const FillNode& node_p ){
    if( this != &node_p ){
      universe = node_p.universe;
      tr = node_p.tr->clone();
    }
    return *this;
  }

  ~FillNode(){
    delete tr;
  }
  
  int getFillingUniverse() const { return universe; }
  
  bool hasTransform() const{ return tr->hasData();}
  const Transform& getTransform() const { return tr->getData(); }

  void setTransform( DataRef<Transform>* tr_p ){
    delete tr;
    tr = tr_p;
  }
};

class Lattice{

protected:
  int num_finite_dims;
  Vector3d v1, v2, v3;

  typedef std::pair<int,int> range;
  range v1_range, v2_range, v3_range;

  bool is_fixed;
  std::vector<FillNode> fills;

  Lattice(){}
  Lattice( const FillNode& fill );

  size_t indicesToSerialIndex( int x, int y, int z ) const ;

public:

  int numFiniteDirections() const { return num_finite_dims; }
  Transform getTxForNode( int x, int y, int z ) const ;

  const FillNode& getFillForNode( int x, int y, int z ) const ;

  bool isFixedSize() const { return is_fixed; }  
  range getRangeForDimension( int dim ) const;
  
  friend class CellCardImpl;

};

class Fill{

protected:
  FillNode origin;

public:
  Fill( const FillNode& origin_p ):
    origin(origin_p)
  {}

  virtual ~Fill(){}

  //  virtual kind getKind() const{ return SIMPLE; }
  virtual FillNode& getOriginNode() { return origin; }
  virtual const FillNode& getOriginNode() const { return origin; }
  
};

#endif /* MCNP2IGEOM_GEOMETRY_H */
