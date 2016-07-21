#include "TestEnvironment.h"

int main(){

  TestQuadcopterActions();
  testLoadPerimeterHeuristic();
  testHCost();
  testGetAction();
  testHeadingTo();
  testConstraints();

  return 0;

}
