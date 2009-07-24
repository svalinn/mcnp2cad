#ifndef MCNP2IGEOM_GEOMETRY_H
#define MCNP2IGEOM_GEOMETRY_H

#include <vector>
#include <cmath>
#include <ostream>

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

  Vector3d operator-() const {
    return Vector3d(-v[0], -v[1], -v[2]);
  }

  Vector3d reverse() const { 
    return -(*this);
  }

};

std::ostream& operator<<(std::ostream& str, const Vector3d& v );

class Transform{

protected:
  Vector3d translation;
  bool degree_format, has_rot;
  double rot_x, rot_y, rot_z;

  void set_rots_from_matrix( double raw_matrix[9] );

public:
  Transform():translation(),has_rot(false){}
  Transform( const std::vector< double >& inputs, bool degree_format_p = false );
  const Vector3d& getTranslation() const { return translation; }
  bool hasRot() const{ return has_rot; }
  double getRotX() const { return rot_x; }
  double getRotY() const { return rot_y; }
  double getRotZ() const { return rot_z; }
  void print( std::ostream& str ) const;

};

class FillNode {

protected:
  int universe;
  DataRef<Transform>* tr;
  bool tr_fixed;

public:
  FillNode():
    universe(0), tr(new NullRef<Transform>()), tr_fixed(false)
  {}

  FillNode( int universe_p, DataRef<Transform>* tr_p, bool tr_fixed_p = false ):
    universe(universe_p), tr(tr_p), tr_fixed(tr_fixed_p)
  {}

  FillNode( const FillNode& node_p ):
    universe(node_p.universe), tr(node_p.tr->clone()), tr_fixed(node_p.tr_fixed)
  {}

  FillNode& operator=( const FillNode& node_p ){
    if( this != &node_p ){
      universe = node_p.universe;
      tr = node_p.tr->clone();
      tr_fixed = node_p.tr_fixed;
    }
    return *this;
  }

  ~FillNode(){
    delete tr;
  }
  
  int getFillingUniverse() const { return universe; }
  
  bool hasTransform() const{ return tr->hasData();}
  const Transform& getTransform() const { return tr->getData(); }
  bool isTransformFixed() const{ return tr_fixed; }

  void setTransform( DataRef<Transform>* tr_p ){
    if(!tr_fixed){
      delete tr;
      tr = tr_p;
    }
  }
};

#include <iostream>

class Fill{

public:
  typedef enum{ SIMPLE, INFINITE, EXPLICIT } kind;
  FillNode origin;

public:
  Fill( const FillNode& origin_p ):
    origin(origin_p)
  {}

  virtual ~Fill(){}

  virtual kind getKind() const{ return SIMPLE; }
  //virtual FillNode& getOriginNode() { return origin; }
  virtual const FillNode& getOriginNode() const { return origin; }
						
  void imbueTransform( const Transform& tr_p ){
    if( !origin.isTransformFixed() ){
      origin.setTransform( new ImmediateRef<Transform>(tr_p) );
    }
    else{
      std::cout << "Refusing to imbue! ... ";
      origin.getTransform().print(std::cout);
      std::cout << std::endl;
    }
  }

};

#endif /* MCNP2IGEOM_GEOMETRY_H */
