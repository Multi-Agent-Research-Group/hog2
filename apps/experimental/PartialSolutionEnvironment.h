/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
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

#ifndef PARTIALSOLUTIONENVIRONMENT_H
#define PARTIALSOLUTIONENVIRONMENT_H

#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <cassert>
#include <vector>


template <class Environment, typename state>
class PartialSolutionEnvironment : public Environment
{
public:
        PartialSolutionEnvironment(Environment* baseEnv, std::vector<state> const& fixedPortion, Heuristic<state> * heur):Environment(baseEnv->GetMapEnv()),base(baseEnv),fixed(fixedPortion),heuristic(heur){
          std::reverse(fixed.begin(),fixed.end()); // Reverse the order.
          for(auto const& a:fixed){
            lookup[GetStateHash(a)]=a;
          }
        }
        virtual std::string name()const{return std::string("PartialSolutionEnvironment");}
        virtual void GetSuccessors(const xyztLoc &node, std::vector<xyztLoc> &neighbors) const{
          if(node.t<fixed.front().t){base->GetSuccessors(node,neighbors);}
          for(auto v(fixed.begin());v!=fixed.end()-1;++v){
            if(node==*v){
              neighbors.push_back(*(v+1));
              return;
            }
          }
        }
	virtual double HCost(const xyztLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for PartialSolutionEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyztLoc &node1, const xyztLoc &node2) const{return lookup.find(GetStateHash(node1))!=lookup.end()?0.0:heuristic->HCost(node1,node2);}
	virtual double GCost(const xyztLoc &node1, const xyztLoc &node2) const{return lookup.find(GetStateHash(node1))!=lookup.end()?0.0:base->GCost(node1,node2);}
	virtual bool LineOfSight(const xyztLoc &node, const xyztLoc &goal) const{return false;}
	virtual bool LineOfSight(const std::pair<xyztLoc,xyztLoc> &node, const std::pair<xyztLoc,xyztLoc> &goal) const{return false;}
	bool GoalTest(const xyztLoc &node, const xyztLoc &goal) const{return lookup.find(GetStateHash(node))!=lookup.end();}
	bool GoalTest(const xyztLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for PartialSolutionEnvironment\n");
		exit(1); return false;}

        virtual state const& getGoal()const{return fixed[0];}

	uint64_t GetStateHash(const xyztLoc &node) const{return *((uint64_t*)&node);}
	virtual void OpenGLDraw() const{}
	virtual void OpenGLDraw(const xyztLoc &l) const{}
	virtual void OpenGLDraw(const xyztLoc &l1, const xyztLoc &l2, float v) const{}
	virtual void GLLabelState(const xyztLoc &, const char *) const{}
	virtual void GLLabelState(const xyztLoc &s, const char *str, double scale) const{}
	virtual void GLDrawLine(const xyztLoc &x, const xyztLoc &y) const{}

	virtual void Draw() const{}
	virtual void Draw(const xyztLoc &l) const{}
	virtual void DrawLine(const xyztLoc &x, const xyztLoc &y, double width = 1.0) const{}

	
	bool IsGoalStored() const {return fixed.size();}
protected:
	Heuristic<state>* heuristic;
        Environment* base;
        std::vector<state> fixed;
        std::unordered_map<uint64_t,state> lookup;
};

#endif
