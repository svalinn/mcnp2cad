#include <eigen3/Eigen/Eigen>
#include <cfloat>
#include <iostream>
#include "GQ_Characterize.hpp"

GQ_Characterize::GQ_Characterize(double A, double B, double C,
                                 double D, double E, double F,
                                 double G, double H, double J,
                                 double K):
    A_(A),B_(B),C_(C),D_(D),E_(E),F_(F),G_(G),H_(H),J_(J),K_(K) {
    //determine canonical form of GQ and determine transformation
    make_canonical();
}

// The GQ Characterization method used here is based on work from
// Skip Thompson at Radford university
// https://www.radford.edu/~thompson/RP/quadrics.pdf
void GQ_Characterize::make_canonical()
{
  // create coefficient matrix
  Eigen::Matrix3f Aa;
  Aa << A_, D_/2, F_/2,
        D_/2, B_, E_/2,
        F_/2, E_/2, C_;

  // create hessian matrix
  Eigen::Matrix4f Ac;
  Ac << A_, D_/2, F_/2, G_/2,
        D_/2, B_, E_/2, H_/2,
        F_/2, E_/2, C_, J_/2,
        G_/2,  H_/2, J_/2, K_;

  // characterization values
  int rnkAa, rnkAc, delta, S, D;
  Eigen::FullPivLU<Eigen::Matrix3f> lu_decomp_Aa(Aa);
  Eigen::FullPivLU<Eigen::Matrix4f> lu_decomp_Ac(Ac);
  rnkAa = lu_decomp_Aa.rank();
  rnkAc = lu_decomp_Ac.rank();

  double determinant = Ac.determinant();
  if (fabs(determinant) < gq_tol)
    delta = 0;
  else
    delta = (determinant < 0) ? -1 : 1;

  Eigen::Vector3f eigenvals;
  Eigen::Matrix3f eigenvects;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> AaEigs;
  // completes Eiegen value computation, stored in AaEigs
  AaEigs.compute(Aa, Eigen::ComputeEigenvectors);
  eigenvals = AaEigs.eigenvalues();
  eigenvects = AaEigs.eigenvectors();
  Eigen::Vector3f signs;

  // determine signs of the eigenvalues
  for(unsigned int i = 0; i < 3; i++) {
    if (eigenvals[i] > -1 * gq_tol)
      signs[i] = 1;  //Sign counted as + if above -tolerance
    else
      signs[i] = -1; //Sign counted as - if less than -tolerance
  }

  // set S parameter based on agreement of eigenvalue signs
  S = (fabs(signs.sum()) == 3) ? 1 : -1;

  // may need to adjust delta for speical cases using the new scaling factor, K_
  // so we'll calculate that now
  Eigen::Vector3f b;
  b << -G_/2, -H_/2, -J_/2;

  // Use a Moore-Penrose pseudoinverse to ensure minimal norm least squares solution
  // this particular inverse ensures we get the "minimal" translation of the GQ
  // surface. A standard inverse may get the correct new basis set (rotation)
  // for the GQ, but with arbitrary scaling and in turn an arbitrarily scaled
  // translation vector
  Eigen::Matrix3f Aai = Aa.completeOrthogonalDecomposition().pseudoInverse();

  // Compute the translation from the inverse
  Eigen::Vector3f c = Aai * b;
  double dx = c[0], dy = c[1], dz = c[2];

  // Update the constant using the resulting translation
  K_ += (G_/2) * dx + (H_/2) * dy + (J_/2) * dz;

  // For the special case of the elliptic cylinder, delta will be needed.
  // Delta is set based on whether the sign of the equation constant, K, and the
  // signs of the eigenvalues are the same.
  if (rnkAa == 2 && rnkAc == 3 && S == 1) {
    delta = (K_ * signs[0]) ? -1 : 1;
  }

  D = (K_*signs[0]) ? -1:1;

  // characterize this GQ equation using the parameter values calculated above
  type = find_type(rnkAa, rnkAc, delta, S, D);
  // set the translation
  translation = Vector3d(dx,dy,dz);
  // set the rotaion matrix. LINE BELOW MAY BE UNNECESSARY but is saved in case
  // it is needed
  std::copy(eigenvects.data(),eigenvects.data()+9,rotation_mat);

  // update equation coefficients
  for(unsigned int i = 0; i < 3; i ++ ) if (fabs(eigenvals[i]) < gq_tol) eigenvals[i] = 0;
  A_ = eigenvals[0]; B_ = eigenvals[1]; C_ = eigenvals[2];
  D_ = 0; E_ = 0; F_ = 0;
  G_ = 0; H_ = 0; J_ = 0;
  // K is set above

  // simplify the GQ if possible
  reduce_type();
}

// this method reduces a complex GQ to a geometrically equivalent
// and more CAD-friendly form if appropriate
void GQ_Characterize::reduce_type() {

  if( ONE_SHEET_HYPERBOLOID == type ) {
    // if the K value is near-zero, reduce to Elliptic Cone
    if ( fabs(K_) < equivalence_tol ) {
      K_ = 0;
      type = ELLIPTIC_CONE;
      return;
    }
  }

  if ( TWO_SHEET_HYPERBOLOID == type ) {
    // if the K value is near-zero, reduce to Elliptic Cone
    if ( fabs(K_) < equivalence_tol ) {
      K_ = 0;
      type = ELLIPTIC_CONE;
      return;
    }
  }

  if ( ELLIPSOID == type ) {
    //if any of the 2nd order terms are near-zero, reduce to Elliptic Cylinder
    if ( fabs(A_) < equivalence_tol ) {
      A_ = 0;
      type = ELLIPTIC_CYL;
      return;
    }
    else if ( fabs(B_) < equivalence_tol ) {
      B_ = 0;
      type = ELLIPTIC_CYL;
      return;
    }
    else if ( fabs(C_) < equivalence_tol ) {
      C_ = 0;
      type = ELLIPTIC_CYL;
      return;
    }
  }

};

GQ_TYPE GQ_Characterize::find_type(int rt, int rf, int del, int s, int d) {

  GQ_TYPE t;
  if( 3 == rt && 4 == rf && -1 == del && 1 == s){
    t = ELLIPSOID;
  }
  else if( 3 == rt && 4 == rf && 1 == del && -1 == s){
    t = ONE_SHEET_HYPERBOLOID;
  }
  else if( 3 == rt && 4 == rf && -1 == del && -1 == s){
    t = TWO_SHEET_HYPERBOLOID;
  }
  else if( 3 == rt && 3 == rf && 0 == del && -1 == s){
    t = ELLIPTIC_CONE;
  }
  else if( 2 == rt && 4 == rf && -1 == del && 1 == s){
    t = ELLIPTIC_PARABOLOID;
  }
  else if( 2 == rt && 4 == rf && 1 == del && -1 == s){
    t = HYPERBOLIC_PARABOLOID;
  }
  else if( 2 == rt && 3 == rf && -1 == del && 1 == s){
    t = ELLIPTIC_CYL;
  }
  else if( 2 == rt && 3 == rf && 0 == del && -1 == s){
    t = HYPERBOLIC_CYL;
  }
  else if( 1 == rt && 3 == rf && 0 == del && 1 == s){
    t = PARABOLIC_CYL;
  }
  else{
    t = UNKNOWN;
  }

  //special case, replace delta with D
  if(2 == rt && 3 == rf && 1 == s && d != 0) {
    t = find_type(rt, rf, d, s, 0);
  }

  return t;
}
