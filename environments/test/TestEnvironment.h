#ifndef __hog2_glut__TestEnvironment__
#define __hog2_glut__TestEnvironment__

#include "../Airplane.h"
#include <iostream>

bool testGetAction(){
   std::cout << "testGetAction()";
   AirplaneEnvironment env;
   airplaneAction a(0,0,0);
   airplaneState s;
   s.x = 50;
   s.y = 50;
   s.height = 26;
   s.heading = 0;
   s.speed = 1;
   std::vector<airplaneAction> actions;
   env.GetActions(s,actions);
   for (auto &a : actions)
   {
     airplaneState s1=s;
     env.ApplyAction(s,a);
     airplaneAction a2(env.GetAction(s1,s));
     //std::cout << a << "=?"<<a2<<"\n";
     assert(a==a2);
     std::cout << ".";
     s=s1; // Reset
   }
   std::cout << "PASSED\n";
}

bool testHeadingTo(){
  std::cout << "testHeadingTo()";
  airplaneState s1(10,10,0,0,0);
  airplaneState s2(10,9,0,0,0);
  assert(s1.headingTo(s2)==0); // Straight down
  std::cout << ".";

  s2.x=11;
  assert(s1.headingTo(s2)==1); // Straight down
  std::cout << ".";

  s2.y=10;
  assert(s1.headingTo(s2)==2); // Straight down
  std::cout << ".";

  s2.y=11;
  assert(s1.headingTo(s2)==3); // Straight down
  std::cout << ".";

  s2.x=10;
  assert(s1.headingTo(s2)==4); // Straight down
  std::cout << ".";

  s2.x=9;
  assert(s1.headingTo(s2)==5); // Straight down
  std::cout << ".";

  s2.y=10;
  assert(s1.headingTo(s2)==6); // Straight down
  std::cout << ".";

  s2.y=9;
  assert(s1.headingTo(s2)==7); // Straight down
  std::cout << ".";

  std::cout << "PASSED\n";
}

#endif
