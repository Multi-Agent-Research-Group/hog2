#include "TestEnvironment.h"

int main(){

  TestQuadcopterActions();
  testGetAction();
  testLoadPerimeterHeuristic();
  testHCost();
  testHeadingTo();
  testConstraints();

  return 0;

}
