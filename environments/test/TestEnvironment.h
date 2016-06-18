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
     std::cout << a << "=?"<<a2<<"\n";
     assert(a==a2);
     std::cout << ".";
     s=s1; // Reset
   }
   std::cout << "PASSED\n";
}

#endif
