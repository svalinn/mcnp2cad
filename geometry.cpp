#include "geometry.hpp"
#include <cfloat>
#include <iostream>

std::ostream& operator<<(std::ostream& str, const Vector3d& v ){
  str << "(" << v.v[0] << ", " << v.v[1] << ", " << v.v[2] << ")";
  return str;
}

// FIXME: assumes M_PI defined; not actually part of standard library everywhere.

/**
 * Compute Euler axis/angle, given a rotation matix.
 * See en.wikipedia.org/wiki/Rotation_representation_(mathematics) 
 */
void Transform::set_rots_from_matrix( double raw_matrix[9] ){
  //double mat[3][3] = {{ raw_matrix[0], raw_matrix[1], raw_matrix[2] },
  // 		      { raw_matrix[3], raw_matrix[4], raw_matrix[5] },
  // 		      { raw_matrix[6], raw_matrix[7], raw_matrix[8] } };
  
  double mat[3][3] = {{ raw_matrix[0], raw_matrix[3], raw_matrix[6] },
  		      { raw_matrix[1], raw_matrix[4], raw_matrix[7] },
  		      { raw_matrix[2], raw_matrix[5], raw_matrix[8] } };
  
  theta = acos( (mat[0][0] + mat[1][1] + mat[2][2] - 1) / 2 );
  double twoSinTheta = 2 * sin(theta);
  axis.v[0] = ( mat[2][1]-mat[1][2]) / twoSinTheta;
  axis.v[1] = ( mat[0][2]-mat[2][0]) / twoSinTheta;
  axis.v[2] = ( mat[1][0]-mat[0][1]) / twoSinTheta;

  // convert theta back to degrees, as used by iGeom
  theta *= 180.0 / M_PI;

}

Transform::Transform( const std::vector< double >& inputs, bool degree_format_p ) : 
  has_rot(false)
{
  
  size_t num_inputs = inputs.size();
  
  // translation is always defined by first three inputs
  translation = Vector3d(inputs); 

  if( num_inputs == 9 ||                         // translation matrix with third vector missing
      num_inputs == 12 || num_inputs == 13 )  // translation matrix fully specified
    {
    
      has_rot = true;
      double raw_matrix[9];
    
    if( num_inputs == 9 ){
      for( int i = 3; i < 9; ++i){
	raw_matrix[i-3] = degree_format_p ? cos(inputs.at(i) * M_PI / 180.0 ) : inputs.at(i);
      }
      
      Vector3d v1( raw_matrix ); //v1 = v1.normalize();
      Vector3d v2( raw_matrix+3 ); //v2 = v2.normalize();
      Vector3d v3 = v1.cross(v2);//.normalize();
      raw_matrix[6] = v3.v[0];
      raw_matrix[7] = v3.v[1];
      raw_matrix[8] = v3.v[2];
    }
    else{
      for( int i = 3; i < 12; ++i){
	raw_matrix[i-3] = degree_format_p ? cos(inputs.at(i) * M_PI / 180.0 ) : inputs.at(i);
      }
      if( num_inputs == 13 && inputs.at(12) == -1.0 ){
	std::cout << "Notice: a transformation has M = -1.  Inverting the translation;" << std::endl;
	std::cout << " though this might not be what you wanted." << std::endl;
	translation = -translation;
      }
    }

    set_rots_from_matrix( raw_matrix);
    
  }
  else if( num_inputs != 3 ){
    // an unsupported number of transformation inputs
    std::cerr << "Warning: transformation with " << num_inputs << " input items is unsupported" << std::endl;
    std::cerr << "  (will pretend there's no rotation: expect incorrect geometry.)" << std::endl;
  }
  
}  

Transform Transform::reverse() const {
  Transform t;
  t.translation = -this->translation;
  t.has_rot = this->has_rot;
  t.axis = -this->axis;
  t.theta = this->theta;
  return t;
}

void Transform::print( std::ostream& str ) const{
  str << "[trans " << translation;
  if(has_rot){
    str << "(" << theta << ":" << axis << ")";
  }
  str << "]";
}

Lattice::Lattice( const FillNode& fill ) : 
  num_finite_dims(0), is_fixed(false)
{
  fills.push_back(fill);
}


// size_t indicesToSerialIndex( int x, int y, int z );


Transform Lattice::getTxForNode( int x, int y, int z ) const {

  Vector3d v;
  switch( num_finite_dims ){
  case 3:
    v = v3 * z; // fallthrough
  case 2:
    v = v + v2 * y; // fallthrough
  case 1:
    v = v + v1 * x;
  default:
    break;
  }

  std::cout << v << x << ", " << y << ", " << z << std::endl;

  return Transform(v);
}

const FillNode& Lattice::getFillForNode( int /*x*/, int /*y*/, int /*z*/ ) const {
  if( is_fixed ){
    throw std::runtime_error("is_Fixed!");
  }
  else{
    return fills.at(0);
  }
}

  
