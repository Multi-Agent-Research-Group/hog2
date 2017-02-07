#include "AirplanePerimeterBuilder.h"
#include "Airplane.h"
#include "AirplaneGridless.h"
#include <gtest/gtest.h>

template<typename environ, typename state>
void TestReverseSuccessors(environ& env, state const& s, state const& g, state o) {
  //std::cout << "original " << o << "\n";
  std::vector<state> ancestors;
  env.GetReverseSuccessors(o,ancestors);
  for (auto &a : ancestors)
  {
    //std::cout << " Ancestor " << a << "\n";
    // All ancestors should have a successor that is equal to the original state
    std::vector<state> successors;
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

TEST(GridlessEnvironmentTest, GetActions) { 
  AirplaneGridlessEnvironment env;
  PlatformAction a(0,0,0);
  PlatformState s(50,50,16,0,0,3);
  PlatformState g(50,40,18,0,0,3);
  env.setGoal(g);
  std::vector<PlatformAction> actions;
  env.GetActions(s,actions);
  std::vector<PlatformAction> ractions;
  env.GetReverseActions(s,ractions);
  ASSERT_EQ(27, actions.size());
  ASSERT_EQ(27, ractions.size());

  for (auto &a : actions)
  {
    PlatformState s1=s;
    env.ApplyAction(s,a);
    ASSERT_FALSE(s==s1);
    PlatformAction a2(env.GetAction(s1,s));
    std::cout <<a;
    std::cout <<a2;
    env.UndoAction(s,a);
    ASSERT_EQ(s,s1);
    ASSERT_EQ(a,a2);
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  PlatformState o(56,44,11,0,0,3);
  TestReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
}

TEST(HeuristicTest, LearnSpace) { 
  AirplaneEnvironment env;
  env.setGoal(airtimeState());
  AirplanePerimeterBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
  std::vector<airplaneState> states;
  states.emplace_back(40,40,10,3,0);
  builder.learnSpace(env,states,5);
  airplaneState s(40,39,10,3,0);
  airplaneState s2(40,41,10,3,0);
  std::cout << env.GCost(s,states[0]) << "\n";
  std::cout << builder.GCost(s,states[0],env) << "\n";
  std::cout << builder.GCost(s2,states[0],env) << "\n";
}
