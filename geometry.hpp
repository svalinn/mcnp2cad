#ifndef MCNP2IGEOM_GEOMETRY_H
#define MCNP2IGEOM_GEOMETRY_H

#include <vector>
#include <cmath>
#include <ostream>

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

// temporary
#include <vector>

class Transform{

protected:
  Vector3d translation;
  bool degree_format, has_rot;
  double rot_x, rot_y, rot_z;

  void set_rots_from_matrix( double raw_matrix[9] );

public:
  Transform(){}
  Transform( const std::vector< double >& inputs, bool degree_format_p = false );
  const Vector3d& getTranslation() const { return translation; }
  bool hasRot() const{ return has_rot; }
  double getRotX() const { return rot_x; }
  double getRotY() const { return rot_y; }
  double getRotZ() const { return rot_z; }
  void print( std::ostream& str ) const;

};

#endif /* MCNP2IGEOM_GEOMETRY_H */
