#include <eigen3/Eigen/Eigen>
#include <cfloat>
#include <iostream>
#include "GQ_Characterize.hpp"

  // tolerance used to determine
  // if matrix determinant should be considered zero

  GQ_Characterize::GQ_Characterize(double A, double B, double C, double D, double E, double F, double G, double H, double J, double K):
    A_(A),B_(B),C_(C),D_(D),E_(E),F_(F),G_(G),H_(H),J_(J),K_(K) {
    //determine canonical form of GQ and determine transformation
    make_canonical();
  }

  void GQ_Characterize::make_canonical()
  {
  //create coefficient matrix
  Eigen::Matrix3f Aa;
  Aa << A_, D_/2, F_/2,
        D_/2, B_, E_/2,
        F_/2, E_/2, C_;
  //create hessian matrix
  Eigen::Matrix4f Ac;
  Ac << A_, D_/2, F_/2, G_/2,
        D_/2, B_, E_/2, H_/2,
        F_/2, E_/2, C_, J_/2,
        G_/2,  H_/2, J_/2, K_;

  //characterization values
  int rnkAa, rnkAc, delta, S, D;
  Eigen::FullPivLU<Eigen::Matrix3f> lu_decomp_Aa(Aa);
  Eigen::FullPivLU<Eigen::Matrix4f> lu_decomp_Ac(Ac);
  rnkAa = lu_decomp_Aa.rank();
  rnkAc = lu_decomp_Ac.rank();

  double determinant = lu_decomp_Ac.determinant();
  if (fabs(determinant) < gq_tol)
    delta = 0;
  else
    delta = (determinant < 0) ? -1:1;

  Eigen::Vector3f eigenvals;
  Eigen::Matrix3f eigenvects;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> AaEigs;
  AaEigs.compute(Aa, Eigen::ComputeEigenvectors); //Computes eig-stuff, stored in AaEigs
  eigenvals = AaEigs.eigenvalues();
  eigenvects = AaEigs.eigenvectors();
  Eigen::Vector3f signs;

  for(unsigned int i = 0; i < eigenvals.diagonalSize(); i++) {
    if (fabs(eigenvals[i]) < gq_tol)
      signs[i] = 1;
    else if (eigenvals[i] > 0)
      signs[i] = 1;
    else if (eigenvals[i] < 0)
      signs[i] = -1;
  }

  S = (fabs(signs(0)+signs(1)+signs(2)) == 3) ? 1:-1;
  // may need to adjust delta for speical cases using the new scaling factor, K_
  // so we'll calculate that now
  Eigen::Vector3f b;
  b << -G_/2, -H_/2, -J_/2;
  //use Moore-Penrose pseudoinverse to ensure minimal norm least squares solution
  //arma::mat Aai = pinv(Aa); //Original code; Working on the 3x3 Coefficient matrix
  double pinvToler = 1.e-6; //Tolerance; how close to 0 is "0"?
  //Remember to change pinvToler to work based on expected input values

  Eigen::JacobiSVD<Eigen::Matrix3f> m_singularValues(Aa, Eigen::ComputeFullU|Eigen::ComputeFullV);
  //ComputeFullU and ComputeFullV tell it to specifically get U and V ready
  Eigen::Matrix3f m_matrixV = m_singularValues.matrixV();
  Eigen::Matrix3f m_matrixU = m_singularValues.matrixU();
  Eigen::Vector3f sv_list = m_singularValues.singularValues();
  Eigen::Matrix3f singularValues_inv;
  for ( long i=0; i<Aa.cols(); ++i) { //Iterate through each column of Aa (3 iterations)
     if ( sv_list(i) > pinvToler )
        singularValues_inv(i,i)=1.0/sv_list(i); //Invert nonzero SVs one by one
     else singularValues_inv(i,i)=0; //SVs close to zero are not inverted
  }
  Eigen::Matrix3f Aai=(m_matrixV*singularValues_inv*m_matrixU.transpose());

  Eigen::Vector3f c = Aai*b;
  double dx = c[0], dy = c[1], dz = c[2];
  K_ = K_ + (G_/2)*dx + (H_/2)*dy + (J_/2)*dz;
  if (rnkAa == 2 && rnkAc == 3 && S == 1)
  delta = ((K_ < 0 && signs[0] < 0) || (K_ > 0 && signs[0] > 0)) ? -1:1;
  D = (K_*signs[0]) ? -1:1;
  //based on characteristic values, get the GQ type
  type = find_type(rnkAa,rnkAc,delta,S,D);
  //set the translation while we're at it
  translation = Vector3d(dx,dy,dz);
  //set the rotaion matrix. LINE BELOW MAY BE UNNECESSARY but is saved in case it is needed
  //std::copy(eigenvects.memptr(),eigenvects.memptr()+9,rotation_mat);
  //set the new canonical values
  for(unsigned int i = 0; i < 3; i ++ ) if (fabs(eigenvals[i]) < gq_tol) eigenvals[i] = 0;
  A_ = eigenvals[0]; B_ = eigenvals[1]; C_ = eigenvals[2];
  D_ = 0; E_ = 0; F_ = 0;
  G_ = 0; H_ = 0; J_ = 0;
  //K is set above

  // simplify the GQ if possible
  reduce_type();
  record << "GQ Type is: " << type << std::endl;
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
    if( 3 == rt && 4 == rf && -1 == del && 1 == s)
      t = ELLIPSOID;
    else if( 3 == rt && 4 == rf && 1 == del && -1 == s)
      t = ONE_SHEET_HYPERBOLOID;
    else if( 3 == rt && 4 == rf && -1 == del && -1 == s)
      t = TWO_SHEET_HYPERBOLOID;
    else if( 3 == rt && 3 == rf && 0 == del && -1 == s)
      t = ELLIPTIC_CONE;
    else if( 2 == rt && 4 == rf && -1 == del && 1 == s)
      t = ELLIPTIC_PARABOLOID;
    else if( 2 == rt && 4 == rf && 1 == del && -1 == s)
      t = HYPERBOLIC_PARABOLOID;
    else if( 2 == rt && 3 == rf && -1 == del && 1 == s)
      t = ELLIPTIC_CYL;
    else if( 2 == rt && 3 == rf && 0 == del && -1 == s)
      t = HYPERBOLIC_CYL;
    else if( 1 == rt && 3 == rf && 0 == del && 1 == s)
      t = PARABOLIC_CYL;
    else
      t = UNKNOWN;

    //special case, replace delta with D
    if( /*2 == rt && 3 == rf && 1 == s && */ d != 0) {
      t = find_type(rt, rf, d, s, 0);
    }
    cout << t << "\n" //Temporary line, for testing
    return t;
  }
