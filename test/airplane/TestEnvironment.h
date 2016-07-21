#ifndef __hog2_glut__TestEnvironment__
#define __hog2_glut__TestEnvironment__

#include "Airplane.h"
#include "AirplaneSimple.h"
#include "AirplaneConstrained.h"
//#include "AirplanePerimeterDBBuilder.h"
#include <iostream>
#include "TemplateAStar.h"
#include "Heuristic.h"

bool testLoadPerimeterHeuristic(){
  // Simple Env.
  {
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
    AirplaneSimpleEnvironment env;
    env.loadPerimeterDB();
    airplaneState target(10,10,10,1,0);
    builder.loadGCosts(env, target, "airplaneSimplePerimiter.dat");

    airplaneState s;
    s.x = 40;
    s.y = 40;
    s.height = 14;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);
    ZeroHeuristic<airplaneState> z;

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(s,g) << "\n";}
      assert(fequal(gcost,builder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

  }
  {
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
    AirplaneEnvironment env;
    env.loadPerimeterDB();
    airplaneState target(10,10,10,1,0);
    builder.loadGCosts(env, target, "airplanePerimiter.dat");

    airplaneState s;
    s.x = 40;
    s.y = 40;
    s.height = 14;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);
    ZeroHeuristic<airplaneState> z;

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(s,g) << "\n";}
      assert(fequal(gcost,builder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

  }

  std::cout << "PASSED\n";
}

bool testHCost(){
  std::cout << "testHCost()";
  {
    std::cout << " Normal Environment: ";
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
        assert(fequal(gcost,env.HCost(s,g)));
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
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fgeq(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

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
        assert(fgeq(gcost,env.HCost(g2,s)));
        std::cout << "." << std::flush;
      }
    }

  }
  {
    std::cout << " Simple Environment: ";
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
      {
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g,s)))
        {
          std::cout << "\n";
          std::cout << "  " << *(sol.begin()) <<"\n";
          for(auto a(sol.begin()+1); a!=sol.end(); ++a)
            std::cout << "  " << *a << " " <<env.GCost(*(a-1),*a)<<"\n";
          std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
        }
        assert(fgeq(gcost,env.HCost(g,s)));
      }
      std::cout << "." << std::flush;
    }

  }
  std::cout << "PASSED\n";
}

bool testGetAction(){
   std::cout << "testGetAction()";
   {
     std::cout << " simple:";
     AirplaneSimpleEnvironment env;
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
       env.UndoAction(s,a);
       //std::cout << a << s << s1 << "\n";
       assert(s==s1 && "Action not reversed properly");
       assert(a==a2 && "Action not inferred properly");
       std::cout << ".";
       s=s1; // Reset
     }
   }
   {
     std::cout << " normal:";
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
       env.UndoAction(s,a);
       //std::cout << a << s << s1 << "\n";
       assert(s==s1 && "Action not reversed properly");
       assert(a==a2 && "Action not inferred properly");
       std::cout << ".";
       s=s1; // Reset
     }
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

bool TestQuadcopterActions() {
  std::cout << "TestQuadcopterActions()";
  AirplaneEnvironment ae;

  airplaneState x(40,40,8,0,0,false,AirplaneType::QUAD);
  std::vector<airplaneAction> actions;
  ae.GetActions(x, actions);

  bool ns = false;

  for (auto a : actions) {
    airplaneState y(40,40,8,0,0,false,AirplaneType::QUAD);
    ae.ApplyAction(y, a);
    std::vector<airplaneAction> backacts;
    ae.GetActions(y, backacts);
    bool valid = false;
    for (auto b : backacts) {
      airplaneState g = y;
      ae.ApplyAction(g, b);
      if (x.x == g.x && x.y == g.y && x.height == g.height){
        valid = true;
        break;
      }
    }
    if (!valid) {
      assert(!"Broken actions in Quadricopter....");
    }
    std::cout <<".";

    if (y.height == 9 && y.x == 40 && y.y == 40)
      ns = true;

  }

  if (!ns) {
    assert(!"Error going up.");
  }

  std::cout << "PASSED\n";


}

#endif
