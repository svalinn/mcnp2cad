#include "GQ_Characterize.hpp"
#include <iostream>
#include <sstream>

int check_gq_type(int test_num, GQ_Characterize* gq, GQ_TYPE expected_type, std::stringstream& err_msg) {

  if (gq->get_type() != expected_type) {
    err_msg << "ERROR IN TEST #" << test_num << ":" << "\n";
    err_msg << "\t" << "Expected type: " << _gq_names[expected_type] << "\n"
              << "\t" << "Actual type: " << _gq_names[gq->get_type()] << "\n";
    return 1;
  } else {
    return 0;
  }
}

int main(){

  int failed_tests = 0;
  std::stringstream err_msg;

// All unit tests currently successful when special case line is fully decommented
  //Should print ELLIPSOID
  GQ_Characterize* testEllip = new GQ_Characterize(1.0,1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
  failed_tests += check_gq_type(1, testEllip, GQ_TYPE::ELLIPSOID, err_msg);
  delete testEllip;

  //ONE_SHEET_HYPERBOLOID
  GQ_Characterize* testOneSheet = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
  failed_tests += check_gq_type(2, testOneSheet, GQ_TYPE::ONE_SHEET_HYPERBOLOID, err_msg);
  delete testOneSheet;

  //TWO_SHEET_HYPERBOLOID
  GQ_Characterize* testTwoSheet = new GQ_Characterize(-1.0,-1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
  failed_tests += check_gq_type(3, testTwoSheet, GQ_TYPE::TWO_SHEET_HYPERBOLOID, err_msg);
  delete testTwoSheet;

  //ELLIPTIC_CONE
  GQ_Characterize* testEllCone = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0);
  failed_tests += check_gq_type(4, testEllCone, GQ_TYPE::ELLIPTIC_CONE, err_msg);
  delete testEllCone;

  //ELLIPTIC_PARABOLOID
  GQ_Characterize* testEllPara = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
  failed_tests += check_gq_type(5, testEllPara, GQ_TYPE::ELLIPTIC_PARABOLOID, err_msg);
  delete testEllPara;

  //HYPERBOLIC_PARABOLOID
  GQ_Characterize* testHypPara = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
  failed_tests += check_gq_type(6, testHypPara, GQ_TYPE::HYPERBOLIC_PARABOLOID, err_msg);
  delete testHypPara;

  //ELLIPTIC_CYL
  GQ_Characterize* testEllCyl = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
  failed_tests += check_gq_type(7, testEllCyl, GQ_TYPE::ELLIPTIC_CYL, err_msg);
  delete testEllCyl;

  //HYPERBOLIC_CYL
  GQ_Characterize* testHypCyl = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0);
  failed_tests += check_gq_type(8, testHypCyl, GQ_TYPE::HYPERBOLIC_CYL, err_msg);
  delete testHypCyl;

  //PARABOLIC_CYL
  GQ_Characterize* testParaCyl = new GQ_Characterize(1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0);
  failed_tests += check_gq_type(9, testParaCyl, GQ_TYPE::PARABOLIC_CYL, err_msg);
  delete testParaCyl;

//Begin Rotation tests
  //Should return ELLIPSOID
  GQ_Characterize* testRotEllip = new GQ_Characterize(103,125,66,-48,-12,-60,0,0,0,-294);
  failed_tests += check_gq_type(10, testRotEllip, GQ_TYPE::ELLIPSOID, err_msg);
  delete testRotEllip;

  //ELLIPTIC_CONE
  GQ_Characterize* testRotCone = new GQ_Characterize(3,3,-1,2, 0,0,0,0,0,0);
  failed_tests += check_gq_type(11, testRotCone, GQ_TYPE::ELLIPTIC_CONE, err_msg);
  delete testRotCone;

  //ELLIPTIC_PARABOLOID
  GQ_Characterize* testRotEllParab = new GQ_Characterize(1,3,1,2,2,2,-2,4,2,12);
  failed_tests += check_gq_type(12, testRotEllParab, GQ_TYPE::ELLIPTIC_PARABOLOID, err_msg);
  delete testRotEllParab;

  //ELLIPTIC_CYLINDER
  GQ_Characterize* testRotEllCyl = new GQ_Characterize(5,2,5,-4,-2,-4,6,-12,18,3);
  failed_tests += check_gq_type(13, testRotEllCyl, GQ_TYPE::ELLIPTIC_CYL, err_msg);
  delete testRotEllCyl;

  if (failed_tests != 0) {
    std::cout << "Tests Failed: " << failed_tests << "\n";
    std::cout << err_msg.str() << std::endl;
  }

  return failed_tests;
}
