#ifndef __hog2_glut__TestEnvironment__
#define __hog2_glut__TestEnvironment__

#include "../Airplane.h"
#include "../AirplaneSimple.h"
#include "../AirplaneConstrained.h"
#include <iostream>
#include "TemplateAStar.h"
#include "Heuristic.h"

bool testHCost(){
  std::cout << "testHCost()";
  {
    std::cout << "\nNormal Environment\n";
    AirplaneEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,env.HCost(s,g))){
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";}
        assert(fequal(gcost,env.HCost(s,g)));//||fgreater(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      //if(fequal(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }
return 1;

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      for(auto &a2: as){
        airplaneState g2(g);
        env.ApplyAction(g2,a2);
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        astar.SetHeuristic(&z);
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g2,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g2,s)))
        {
          std::cout << "\n";
          for(auto &a : sol)
            std::cout << "  " << a<<"\n";
          std::cout << g2 << s << " G " << gcost << " H " << env.HCost(g2,s) << "\n";
        }
        assert(fequal(gcost,env.HCost(g2,s))||fgreater(gcost,env.HCost(g2,s)));
        std::cout << "." << std::flush;
      }
    }

  }
  {
    std::cout << "\nSimple Environment\n";
    AirplaneSimpleEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      //if(gcost!=env.GCost(s,g))
        //std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";
        assert(fequal(gcost,env.HCost(s,g))||fgreater(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      astar.SetWeight(.5);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> sh;
    ManhattanHeuristic<airplaneState> mh;
    {
      airplaneState s(6,9,8,3,2,false);
      airplaneState g(9,12,6,3,2,false);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));

    }
    for(int i(0); i<1000; ++i){
      airplaneState s(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      airplaneState g(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      if(s==g)continue;
      //std::cout << s << g << std::endl;
      /*{
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        astar.SetHeuristic(&sh);
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g,s)))
        {
          std::cout << "\n";
          for(auto &a : sol)
            std::cout << "  " << a<<"\n";
          std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
        }
        assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      }*/
      {
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g,s)))
        {
          std::cout << "\n";
          for(auto &a : sol)
            std::cout << "  " << a<<"\n";
          std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
        }
        assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      }
      std::cout << "." << std::flush;
    }

  }
  std::cout << "PASSED\n";
}

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

bool testConstraints() {
	std::cout << "testConstraints()";
	AirplaneConstrainedEnvironment ace(new AirplaneEnvironment());

	airplaneState O(0,0,0,0,0);
	airtimeState OT(O, 1.0f);
	airConstraint CO(OT);


	// Check self-conflict - you should always conflict with yourself
	{
		airplaneState s1(10,10,10,0,0);
		airtimeState st1(s1, 1.0f);
		airConstraint c(st1);

		// Should always conflict with yourself
		assert(c.ConflictsWith(c));
		assert(c.ConflictsWith(st1));
		assert(c.ConflictsWith(st1, st1));

		// Should not conflict with this
		assert(!c.ConflictsWith(CO));

		std::cout << ".";
	}

	// Don't conflict if times do not overlap
	{
		airplaneState s1(10,10,10,0,0);
		airtimeState st1(s1, 1.0f);
		airtimeState st2(s1, 2.0f);
		airConstraint c(st1);

		assert(!c.ConflictsWith(st2));

		// Should not conflict with this
		assert(!c.ConflictsWith(CO));

		std::cout << ".";
	}

	// another case..
	{
		airplaneState s1(32,40,14,1,6);
		airplaneState s2(31,41,14,2,6);
		airtimeState st1(s1, 0.0f);
		airtimeState st2(s1, 0.204465f);
		airConstraint c(st1);

		assert(!c.ConflictsWith(st2));

		std::cout << ".";
	}

	// Check simple straight conflict
	{
		// Check over all speeds
		for (int j = 1; j < ace.GetSpeeds(); j++)
		{
			// Check over all headings
			for (int i = 0; i < 8; i++)
			{
				airplaneState s1(10,10,10,j,i);
				airtimeState st1(s1, 1.0f);
				airplaneAction a1(0, 0, 0);
				
				airtimeState st2(s1, 1.0f);
				ace.ApplyAction(st2, a1);
				
				airConstraint c1(st1, st2);

				
				// Should conflict at both endpoints
				assert(c1.ConflictsWith(st2));
				assert(c1.ConflictsWith(st1));

				// Should conflict with the motion
				assert(c1.ConflictsWith(st1, st2));

				// Should also conflict with something in the middle
				airtimeState st3(s1, 0.9f);
				ace.ApplyAction(st3, a1);
				assert(c1.ConflictsWith(st3));

				assert(!c1.ConflictsWith(CO));

				std::cout << ".";
			}
		}
	}

	// Check simple turn conflict
	{
		// Check all turns
		for (int k = -3; k <= 3; k++)
		{
			// Check over all speeds
			for (int j = 1; j < ace.GetSpeeds(); j++)
			{
				// Check over all headings
				for (int i = 0; i < 8; i++)
				{
					airplaneState s1(10,10,10,j,i);
					airtimeState st1(s1, 1.0f);

					airplaneAction a1(k, 0, 0);
					
					airtimeState st2(s1, 1.0f);
					ace.ApplyAction(st2, a1);
					
					airConstraint c1(st1, st2);

					
					// Should conflict at both endpoints
					assert(c1.ConflictsWith(st2));
					assert(c1.ConflictsWith(st1));

					// Should conflict with the motion
					assert(c1.ConflictsWith(st1, st2));

					// Should also conflict with something in the middle
					airtimeState st3(s1, 0.9f);
					ace.ApplyAction(st3, a1);
					assert(c1.ConflictsWith(st3));

					assert(!c1.ConflictsWith(CO));

					std::cout << ".";
				}
			}
		}
	}

	// Check height change conflict
	{
		// Check all turns
		for (int k = -3; k <= 3; k++)
		{
			// Check over all speeds
			for (int j = 1; j < ace.GetSpeeds(); j++)
			{
				// Check over all headings
				for (int i = 0; i < 8; i++)
				{
					airplaneState s1(10,10,10,j,i);
					airtimeState st1(s1, 1.0f);

					airplaneAction a1(k, 0, 1);
					
					airtimeState st2(s1, 1.0f);
					ace.ApplyAction(st2, a1);
					
					airConstraint c1(st1, st2);

					
					// Should conflict at both endpoints
					assert(c1.ConflictsWith(st2));
					assert(c1.ConflictsWith(st1));

					// Should conflict with the motion
					assert(c1.ConflictsWith(st1, st2));

					// Should also conflict with something in the middle
					airtimeState st3(s1, 0.9f);
					ace.ApplyAction(st3, a1);
					assert(c1.ConflictsWith(st3));

					assert(!c1.ConflictsWith(CO));

					std::cout << ".";
				}
			}
		}
	}

	std::cout << "PASSED\n";
}

#endif
