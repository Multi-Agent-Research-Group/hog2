#include "Airplane.h"
#include <gtest/gtest.h>

void TestReverseSuccessors(AirplaneEnvironment& env, airplaneState const& s, airplaneState const& g, airplaneState o) {
  //std::cout << "original " << o << "\n";
  std::vector<airplaneState> ancestors;
  env.GetReverseSuccessors(o,ancestors);
  for (auto &a : ancestors)
  {
    //std::cout << " Ancestor " << a << "\n";
    // All ancestors should have a successor that is equal to the original state
    std::vector<airplaneState> successors;
    env.GetSuccessors(a,successors);
    bool found(false);
    for(auto &c: successors){
      //std::cout << "        " << c << "\n";
      if(c==o){found=true; break;}
    }
    ASSERT_TRUE(found);
  }
}

TEST(AirplaneEnvironmentTest, GetActions) { 
  AirplaneEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s(50,50,16,3,0);
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  ASSERT_EQ(63, actions.size());
  ASSERT_EQ(63, ractions.size());

  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    ASSERT_EQ(s,s1);
    ASSERT_EQ(a,a2);
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  TestReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
}
