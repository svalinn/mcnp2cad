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

  for(unsigned int i = 0; i < 3; i++) {  //Loop once for each of the 3 eigenvalues
                                        //TODO: Change 3 to some "size" variable
    if (eigenvals[i] > -1*gq_tol)
      signs[i] = 1;  //Sign counted as + if above -tolerance
    else
      signs[i] = -1; //Sign counted as - if less than -tolerance
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
  //TODO: Not sure what purpose or origin of 'record' is
  //record << "GQ Type is: " << type << std::endl;
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
    
  GQ_Characterize::GQ_TYPE GQ_Characterize::find_type(int rt, int rf, int del, int s, int d) {
	//TEST CODE (TODO: Remove this when testing is complete)
	std::cout << "rt = " << rt << "\n";
	std::cout << "rf = " << rf << "\n";
        std::cout << "del = " << del << "\n";
	std::cout << "s = " << s << "\n";
	std::cout << "d = " << d << "\n";


    GQ_TYPE t;
    if( 3 == rt && 4 == rf && -1 == del && 1 == s){
      t = ELLIPSOID;
      std::cout << "ELLIPSOID\n";
    }
    else if( 3 == rt && 4 == rf && 1 == del && -1 == s){
      t = ONE_SHEET_HYPERBOLOID;
      std::cout << "ONE_SHEET_HYPERBOLOID\n";
    }
    else if( 3 == rt && 4 == rf && -1 == del && -1 == s){
      t = TWO_SHEET_HYPERBOLOID;
      std::cout << "TWO_SHEET_HYPERBOLOID\n";
    }
    else if( 3 == rt && 3 == rf && 0 == del && -1 == s){
      t = ELLIPTIC_CONE;
      std::cout << "ELLIPTIC_CONE\n";
    }
    else if( 2 == rt && 4 == rf && -1 == del && 1 == s){
      t = ELLIPTIC_PARABOLOID;
      std::cout << "ELLIPTID_PARABOLOID\n";
    }
    else if( 2 == rt && 4 == rf && 1 == del && -1 == s){
      t = HYPERBOLIC_PARABOLOID;
      std::cout << "HYPERBOLIC_PARABOLOID\n";
    }
    else if( 2 == rt && 3 == rf && -1 == del && 1 == s){
      t = ELLIPTIC_CYL;
      std::cout << "ELLIPTIC_CYL\n";
    }
    else if( 2 == rt && 3 == rf && 0 == del && -1 == s){
      t = HYPERBOLIC_CYL;
      std::cout << "HYPERBOLIC_CYL\n";
    }
    else if( 1 == rt && 3 == rf && 0 == del && 1 == s){
      t = PARABOLIC_CYL;
      std::cout << "PARABOLIC_CYL\n";
    }
    else{
      t = UNKNOWN;
      std::cout << "UNKNOWN\n";
    }

    //special case, replace delta with D
    if(2 == rt && 3 == rf && 1 == s && d != 0) {
	std::cout << "Special case: ";
      t = find_type(rt, rf, d, s, 0);
    }
    //std::cout << t << "\n"; //Temporary line, for testing
    return t;
  }
