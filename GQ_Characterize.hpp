#ifndef GQ_CHARACTERIZE_H
#define GQ_CHARACTERIZE_H

#include <eigen3/Eigen/Eigen>
#include <cfloat>
#include <iostream>

class GQ_Characterize{

protected:
  // coefficients of the GQ
  double A_,B_,C_,D_,E_,F_,G_,H_,J_,K_;
  // the cannonical GQ type
  int type;
  // translation from the canoncial GQ to final GQ
  Vector3d translation;
  // tolerance used to determine
  // if matrix determinant should be considered zero
  const double gq_tol = 1e-6;
  const double equivalence_tol = 1e-8;
	
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
  //Constructor
  GQ_Characterize(double A, double B, double C, double D, double E, double F, double G, double H, double J, double K);

protected:
  void make_canonical();

  // this method reduces a complex GQ to a geometrically equivalent
  // and more CAD-friendly form if appropriate
  void reduce_type();
    
  GQ_TYPE find_type(int rt, int rf, int del, int s, int d);
};

#endif
