#include "GQ_Characterize.hpp"

int main(){
	//Should print ELLIPSOID or 1
	GQ_Characterize* testEllip = new GQ_Characterize(1.0,1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,1.0);
	delete testEllip;	

/* TODO: Expand to all tests when the first one is debugged 
	//ONE_SHEET_HYPERBOLOID or 2
	GQ_Characterize* testOneSheet = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,1.0);
	delete testOneSheet;

	//TWO_SHEET_HYPERBOLOID or 3
	GQ_Characterize* testTwoSheet = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testTwoSheet;

	//ELLIPTIC_CONE or 4
	GQ_Characterize* testEllCone = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0);
	delete testEllCone;

	//ELLIPTIC_PARABOLOID or 5
	GQ_Characterize* testEllPara = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
	delete testEllPara;

	//HYPERBOLIC_PARABOLOID or 6
	GQ_Characterize* testHypPara = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
	delete testHypPara;

	//ELLIPTIC_CYL or 7
	GQ_Characterize* testEllCyl = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0);
	delete testEllCyl;

	//HYPERBOLIC_CYL or 8
	GQ_Characterize* testHypCyl = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testHypCyl;

	//PARABOLIC_CYL or 9
	GQ_Characterize* testParaCyl = new GQ_Characterize(1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0);
	delete testParaCyl;
*/

	return 0;
}
