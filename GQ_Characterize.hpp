#ifndef GQ_CHARACTERIZE_H
#define GQ_CHARACTERIZE_H

#include <eigen3/Eigen/Eigen>
#include <cfloat>
#include <iostream>
#include <string>

#include "geometry.hpp"

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

const std::vector<std::string> _gq_names = {"UNKNOWN",
                                            "ELLIPSOID",
                                            "ONE_SHEET_HYPERBOLOID",
                                            "TWO_SHEET_HYPERBOLOID",
                                            "ELLIPTIC_CONE",
                                            "ELLIPTIC_PARABOLOID",
                                            "HYPERBOLIC_PARABOLOID",
                                            "ELLIPTIC_CYL",
                                            "HYPERBOLIC_CYL",
                                            "PARABOLIC_CYL"};

extern const std::vector<std::string> _gq_names;

class GQ_Characterize{

protected:
  // coefficients of the GQ
  double A_,B_,C_,D_,E_,F_,G_,H_,J_,K_;
  // the cannonical GQ type
  GQ_TYPE type;
  // translation from the canoncial GQ to final GQ
  Vector3d translation;
  // rotation matrix from canonical GQ to final GQ
  double rotation_mat[9];
  // gq transform
  Transform transform_;

  // tolerance used to determine
  // if matrix determinant should be considered zero
  const double gq_tol = 1e-6;
  const double equivalence_tol = 1e-8;


public:
  // Constructor
  GQ_Characterize(double, double, double, double, double, double, double, double, double, double);

  // GQ type accessor
  GQ_TYPE get_type() { return type; }

protected:
  // reduces GQ coeficients to 2nd order terms, a constant, and a transformation
  // to the original origntation when needed
  void make_canonical();

  // this method reduces a complex GQ to a geometrically equivalent
  // and more CAD-friendly form if appropriate
  void reduce_type();

  // determines GQ type based on characteristic parameters
  GQ_TYPE find_type(int rt, int rf, int del, int s, int d);
};

#endif
