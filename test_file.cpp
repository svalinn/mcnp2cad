#include "volumes.hpp"

void main()
{
  double A,B,C,G,H,J,K;
  
  //Start with the Parabolic Cylinder
  A = 1.0; B = 0.0; C = 0.0;
  G = 0.0; H = -1.0; J = 0.0;
  K = 0.0;

  GeneralQuadraticSurface newSurf = new GeneralQuadraticSurface(A,B,C,D,E,F,G,H,J,K);
}
