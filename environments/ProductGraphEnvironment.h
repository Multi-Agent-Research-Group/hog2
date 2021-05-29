/*
 *  Created by Thayne Walker, Rajat Kumar Jenamani.
 *  Copyright (c) Thayne Walker 2021 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef ProductGraphEnvironment_h
#define ProductGraphEnvironment_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"

#define VERIFY if(verify_disabled) {} else
bool verify_disabled = false; 

template<typename state>
class MAState : public std::vector<state>{};

// Note state must have fields: source,target,t_value
// Actual Environment
template<typename state, typename action, typename environment>
class ProductGraphEnvironment : public SearchEnvironment<MAState<state>, std::vector<action> >
{
	public:
		//typedef std::vector<state> MAState<state>;
		typedef std::vector<action> MultiAgentAction;

		// Constructor
		ProductGraphEnvironment(std::vector<environment const*> const& base):env(base){
			heuristics.reserve(env.size());
			for(auto const& e:base){
				heuristics.push_back((Heuristic<typename state::first_type> const*)e);
			}
		}
		ProductGraphEnvironment(std::vector<environment const*> const& base, std::vector<Heuristic<typename state::first_type> const*> const& h):env(base),heuristics(h){}

		virtual void GetSuccessors(const MAState<state> &nodeID, std::vector<MAState<state>> &neighbors) const;

		virtual void GetActions(const MAState<state> &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

		virtual void GetReverseActions(const MAState<state> &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

		virtual void ApplyAction(MAState<state> &s, MultiAgentAction dir) const{assert(false);}
		virtual void UndoAction(MAState<state> &s, MultiAgentAction const& dir) const{assert(false);}
		virtual void GetNextState(MAState<state> const& currents, MultiAgentAction const& dir, MAState<state> &news) const{assert(false);}
		virtual bool InvertAction(MultiAgentAction &a) const { return false; }
		virtual MultiAgentAction GetAction(MAState<state> const& node1, MAState<state> const& node2) const{assert(false);}


		// Heuristics and paths
		virtual double HCost(const MAState<state>& node1, const MAState<state>& node2) const;
		virtual double HCost(MAState<state> const& )  const { assert(false); return 0; }
		virtual double GCost(MAState<state> const& node1, MAState<state> const& node2) const;
		virtual double GCost(MAState<state> const& node1, MultiAgentAction const& act) const{assert(false);}
		virtual double GetPathLength(std::vector<MAState<state>> const& n) const;
		void loadPerimeterDB();

		// Goal testing
		virtual bool GoalTest(MAState<state> const& node, MAState<state> const& goal) const;
		virtual bool GoalTest(MAState<state> const&) const { assert(false); return false; }

		// Hashing
		virtual uint64_t GetStateHash(MAState<state> const& node) const;
		virtual uint64_t GetStateHash(state const& node) const{return env[0]->GetStateHash(getPosition(state));}
		virtual uint64_t GetStateHash(unsigned agent, state const& node) const{return env[agent]->GetStateHash(getPosition(node));}
		virtual uint64_t GetActionHash(MultiAgentAction act) const{assert(false);}

		// Drawing
		virtual void OpenGLDraw() const;
		virtual void OpenGLDraw(const MAState<state> &l) const;
		virtual void OpenGLDraw(const MAState<state>& oldState, const MAState<state> &newState, float perc) const;
		virtual void OpenGLDraw(const MAState<state> &, const MultiAgentAction &) const;
		void GLDrawLine(const MAState<state> &a, const MAState<state> &b) const;
		void GLDrawPath(const std::vector<MAState<state>> &p) const;

		MAState<state> const* goal;
		MAState<state> const& getGoal()const{return *goal;}
		void setGoal(MAState<state> const& g){goal=&g;}
		environment const* const getEnv(unsigned index)const{return env[index];}
		double ViolatesConstraint(MAState<state> const& a,MAState<state> const& n){return false;}

	protected:

		//mutable std::vector<MultiAgentAction> internalActions;

		static void generatePermutations(const MAState<state> &s, const std::vector<std::vector<typename state::first_type>>& positions, std::vector<MAState<state>>& result, int agent, MAState<state>& current);
		static void generateCombinations(const std::vector<std::vector<int>>& possible_delta_fs, const int &extra_delta_f,std::vector<std::vector<int>>& result,  int &next_delta_f, int agent, std::vector<int>& current, int &current_extra_delta_f);
	private:
		std::vector<environment const*> env;
		std::vector<Heuristic<typename state::first_type> const*> heuristics;
};

template<typename state>
static std::ostream& operator <<(std::ostream& os, MAState<state> const& s){
	for(auto const& a : s){
		os << "("<<a.source<<","<<a.target<<","<<a.t_value<< ") ";
	}
	return os;
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::generatePermutations(
	const MAState<state> &s, const std::vector<std::vector<typename state::first_type>>& positions, 
	std::vector<MAState<state>>& result, int agent, MAState<state>& current)
{
	static double agentRadius=.25;
	if(agent == positions.size()) 
	{
		int min_timesteps = INT_MAX;
		for(int i=0; i<s.size(); i++)
			min_timesteps = std::min(min_timesteps,getTimeSteps(s[i],current[i]));
		MAState<state> neighbor;
		for(int i=0; i<s.size(); i++)
			neighbor.push_back(executeTimesteps(s[i],current[i],min_timesteps));
		result.push_back(neighbor);
		return;
	}

	if(s[agent].t_value != 0)
	{
		current.emplace_back(s[agent].target,s[agent].target,0); // continue traversing state
		generatePermutations(s,positions, result, agent + 1, current);
	}
	else
	{
		{
			current.push_back(s); // wait at current state
			generatePermutations(s,positions, result, agent + 1, current);
		}
		for(int i = 0; i < positions[agent].size(); ++i) // choose an outedge 
		{
			xytLoc a1 = xytLoc(getPosition(s[agent]),0);
			xytLoc b1 = xytLoc(positions[agent][i],getTimeSteps(t1));
			bool found(false);
			for(int j(0); j<current.size(); ++j)
			{
				xytLoc a2 = xytLoc(getPosition(s[j]), 0);
				xytLoc b2 = xytLoc(getPosition(current[j],getTimeSteps(s[j],current[j])));
				if(collisionCheck3D(a1,b1,a2,b2,agentRadius))
				{
					found=true;
					break;
				}
			}
			if(found) continue;
			MAState<state> copy(current);
			copy.emplace_back(positions[agent][i], positions[agent][i], 0);
			generatePermutations(s,positions, result, agent + 1, copy);
		}
	}
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::generateCombinations(
	const std::vector<std::vector<int>>& possible_delta_fs, const int &extra_delta_f,
	std::vector<std::vector<int>>& result,  int &next_delta_f, 
	int agent, std::vector<int>& current, int &current_extra_delta_f)
{
	if(agent == possible_delta_fs.size()) 
	{
		result.push_back(current);
		return;
	}

	for(int i = 0; i < possible_delta_fs[agent].size(); ++i) 
	{
		int next_extra_delta_f = current_extra_delta_f+(possible_delta_fs[agent][i]- possible_delta_fs[agent][0]);
		if(next_extra_delta_f > extra_delta_f ) 
		{
			next_delta_f = std::min(next_delta_f, next_extra_delta_f);
			continue;
		}
		std::vector<int> copy(current);
		copy.push_back(possible_delta_fs[agent][i]);
		generateCombinations(possible_delta_fs, extra_delta_f,result,next_delta_f, agent + 1, copy, next_extra_delta_f);
	}
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GetDeltaFSuccessors(const MAState<state> &s, const int &delta_f, std::vector<MAState<state>> &neighbors, int &next_delta_f) const
{
	int num_agents = s.size();
	next_delta_f = INT_MAX;
	std::vector<std::vector<int>> possible_delta_fs(num_agents,std::vector<int>());

	for(int i=0; i<num_agents; i++)
		if(s[i].t_value == 0)
			env[k]->GetDeltaFs(s[i].source,delta_f,possible_delta_fs[i]); // possible_delta_fs is sorted
		else
			possible_delta_fs[i].push_back(0); // continue traversing the edge

	int extra_delta_f = delta_f;
	for(int i=0; i<num_agents; i++)
		extra_delta_f -= possible_delta_fs[i][0];
	
	std::vector<std::vector<int>> delta_f_combinations;

	std::vector<int> current;
	int current_extra_delta_f = 0;
	generateCombinations(possible_delta_fs, extra_delta_f,delta_f_combinations,next_delta_f, 0, current, current_extra_delta_f);

	for(int i=0; i<num_agents; i++)
		next_delta_f += possible_delta_fs[i][0]; 

	for(int i=0; i<delta_f_combinations.size(); i++)
	{
		std::vector<std::vector<typename state::first_type>> successors(num_agents,std::vector<typename state::first_type>());
		for(int agent=0; agent<num_agents; agent++)
			if(s[agent].t_value == 0)
				env[agent]->GetDeltaFSuccessors(s[agent].source,delta_f_combinations[i][agent], successors[agent]);
		std::vector<int> tmp;
		generatePermutations(s,successors,neighbors,0,tmp);
	}
}

template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::HCost(const MAState<state> &node1, const MAState<state> &node2) const
{
	double total(0.0);
	for(int i(0); i<node1.size(); ++i)
	{
		total += heuristics[i]->HCost(node1[i].target,node2[i].source);
		if(node1[i].t_value!=0)
			total += getTimeStepsLeft(node1[i]);
		total += node2[i].t_value;
	}
	return total;
}

template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::GCost(MAState<state> const& node1, MAState<state> const& node2) const {
	double total(0.0);
	for(int i(0); i<node1.size(); ++i)
	{
		total += env[i]->GCost(node1[i].target,node2[i].source);
		if(node1[i].t_value!=0)
			total += getTimeStepsLeft(node1[i]);
		total += node2[i].t_value;
	}
	return total;
}

template<typename state, typename action, typename environment>
bool ProductGraphEnvironment<state,action,environment>::GoalTest(const MAState<state> &node, const MAState<state> &goal) const
{
	for(int i(0); i<node.size(); ++i)
	{
		if( node[i].t_value!=0 || !env[i]->GoalTest(node[i].source,goal[i].source))
			return false;
	}
	return true;
}

template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::GetPathLength(const std::vector<MAState<state>> &sol) const
{
	uint32_t cost(0);
	for(int i=0; i<sol.size(); i++)
		for(int j=0; j<sol[i].size()-1; j++)
			cost += getTimeSteps(sol[i][j],sol[i][j+1]);
	return cost;
}

template<typename state, typename action, typename environment>
uint64_t ProductGraphEnvironment<state,action,environment>::GetStateHash(const MAState<state> &node) const
{
	// Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
	uint64_t h(14695981039346656037UL); // Offset basis
	unsigned i(0);
	for(auto const& v : node){
		uint64_t h1(env[i++]->GetStateHash(getPosition(v)));
		uint8_t c[sizeof(uint64_t)];
		memcpy(c,&h1,sizeof(uint64_t));
		for(unsigned j(0); j<sizeof(uint64_t); ++j){
			//hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
			h=h^c[j]; // Xor with octet
			h=h*1099511628211; // multiply by the FNV prime
		}
	}
	return h;
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw() const
{
	env[0]->OpenGLDraw();
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state> &l) const
{
	//TODO: Implement this
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state>& o, const MAState<state> &n, float perc) const
{
	//TODO: Implement this
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state> &, const ProductGraphEnvironment<state,action,environment>::MultiAgentAction &) const
{
	//TODO: Implement this
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GLDrawLine(const MAState<state> &a, const MAState<state> &b) const
{
	//TODO: Implement this
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GLDrawPath(const std::vector<MAState<state>> &p) const
{
	//TODO: Implement this
}

#endif /* MultiAgent_h */
