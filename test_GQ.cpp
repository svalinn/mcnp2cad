#include "GQ_Characterize.hpp"

int main(){

/*All unit tests currently successful when special case line is fully decommented
	//Should print ELLIPSOID
	GQ_Characterize* testEllip = new GQ_Characterize(1.0,1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testEllip;	

	//ONE_SHEET_HYPERBOLOID
	GQ_Characterize* testOneSheet = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testOneSheet;

	//TWO_SHEET_HYPERBOLOID
	GQ_Characterize* testTwoSheet = new GQ_Characterize(-1.0,-1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testTwoSheet;

	//ELLIPTIC_CONE
	GQ_Characterize* testEllCone = new GQ_Characterize(1.0,1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0);
	delete testEllCone;

	//ELLIPTIC_PARABOLOID
	GQ_Characterize* testEllPara = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
	delete testEllPara;

	//HYPERBOLIC_PARABOLOID
	GQ_Characterize* testHypPara = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0);
	delete testHypPara;

	//ELLIPTIC_CYL
	GQ_Characterize* testEllCyl = new GQ_Characterize(1.0,1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0);
	delete testEllCyl;

	//HYPERBOLIC_CYL
	GQ_Characterize* testHypCyl = new GQ_Characterize(1.0,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0);
	delete testHypCyl;

	//PARABOLIC_CYL
	GQ_Characterize* testParaCyl = new GQ_Characterize(1.0, 0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0);
	delete testParaCyl;

*/

//Begin Rotation tests
        //Should return ELLIPSOID
        GQ_Characterize* testRotEllip = new GQ_Characterize(103,125,66,-48,-12,-60,0,0,0,-294);
        delete testRotEllip;

        //ELLIPTIC_CONE
        GQ_Characterize* testRotCone = new GQ_Characterize(3,3,-1,2, 0,0,0,0,0,0);
        delete testRotCone;

        //ELLIPTIC_PARABOLOID
        GQ_Characterize* testRotEllParab = new GQ_Characterize(1,3,1,2,2,2,-2,4,2,12);
        delete testRotEllParab;

        //ELLIPTIC_CYLINDER
        GQ_Characterize* testRotEllCyl = new GQ_Characterize(5,2,5,-4,-2,-4,6,-12,18,3);
        delete testRotEllCyl;

	return 0;
}



