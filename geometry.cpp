#include "geometry.hpp"
#include <cfloat>
#include <iostream>

std::ostream& operator<<(std::ostream& str, const Vector3d& v ){
  str << "(" << v.v[0] << ", " << v.v[1] << ", " << v.v[2] << ")";
  return str;
}

// FIXME: assumes M_PI defined; not actually part of standard library everywhere.

/**
 * Compute x, y, and z rotations for this transformation given the 9-entry matrix format.
 * I took this code from the original mcnp2acis converter.  I'm not convinced it works for
 * all cases, so watch out for broken transformations!
 */
void Transform::set_rots_from_matrix( double raw_matrix[9] ){
  // double mat[3][3] = {{ raw_matrix[0], raw_matrix[1], raw_matrix[2] },
  // 		      { raw_matrix[3], raw_matrix[4], raw_matrix[5] },
  // 		      { raw_matrix[6], raw_matrix[7], raw_matrix[8] } };
  
  double mat[3][3] = {{ raw_matrix[0], raw_matrix[3], raw_matrix[6] },
  		      { raw_matrix[1], raw_matrix[4], raw_matrix[7] },
  		      { raw_matrix[2], raw_matrix[5], raw_matrix[8] } };
  
  if(degree_format){
    for(int i = 0; i<3; ++i){
      for( int j = 0; j<3; ++j){
	mat[i][j] = cos( mat[i][j] * M_PI / 180.0 );
      }
    }
  }
  
  if( fabs( mat[1][0] - 1.0 ) < DBL_EPSILON ){ //mat[1][0] == 1.0
    rot_x = atan2( mat[0][2], mat[2][2] );
    rot_y = M_PI / 2.0;
    rot_z = 0;
  }
  else if( fabs( mat[1][0] - (-1.0) ) < DBL_EPSILON ){ // mat[1][0] == -1.0
    rot_x = atan2( mat[0][2], mat[2][2] );
    rot_y = -M_PI / 2.0;
    rot_z = 0;
  }
  else{
    rot_x = atan2( -mat[2][0], mat[0][0] );
    rot_y = atan2( -mat[1][2], mat[1][1] );
    rot_z = asin( mat[1][0] );
  }
  
  rot_x *= 180.0 / M_PI;
  rot_y *= 180.0 / M_PI;
  rot_z *= 180.0 / M_PI;
  
}

Transform::Transform( const std::vector< double >& inputs, bool degree_format_p ) : 
  degree_format(degree_format_p), has_rot(false) 
{
  
  size_t num_inputs = inputs.size();
  
  // translation is always defined by first three inputs
  translation = Vector3d(inputs); 
  
  if( num_inputs == 12 || num_inputs == 13 ){ // translation matrix fully specified
    
    has_rot = true;
    double raw_matrix[9];
    
    for( int i = 3; i < 12; ++i){
      raw_matrix[i-3] = inputs.at(i);
    }
    if( num_inputs == 13 && inputs.at(12) == -1.0 ){
      std::cout << "Notice: a transformation has M = -1.  Inverting the translation;" << std::endl;
      std::cout << " though this might not be what you wanted." << std::endl;
      translation = -translation;
    }
    
    set_rots_from_matrix(raw_matrix);
    
  }
  else if( num_inputs != 3 ){
    // an unsupported number of transformation inputs
    std::cerr << "Warning: transformation with " << num_inputs << " input items is unsupported" << std::endl;
    std::cerr << "  (will pretend there's no rotation: expect incorrect geometry.)" << std::endl;
  }
  
}  

void Transform::print( std::ostream& str ) const{
  str << "[trans " << translation;
  if(has_rot){
    str << "(" << rot_x << ", " << rot_y << ", " << rot_z << ")";
  }
  str << "]";
}
