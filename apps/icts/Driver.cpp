/*
 * $Id: sample.h,v 1.6 2006/09/18 06:23:39 nathanst Exp $
 *
 *  Driver.h
 *  hog
 *
 *  Created by Thayne Walker on 3/17/17.
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <iostream>
#include <unordered_map>
#include "Map2DEnvironment.h"

int maxDepth;

struct Node{
	static MapEnvironment& env;
	Node(){}
	Node(xyLoc a, int d):n(a),depth(d){}
	xyLoc n;
	uint16_t depth;
	std::vector<Node*> parents;
	std::vector<Node*> successors;
	uint64_t Hash(){return (env.GetStateHash(n)<<16) & depth;}
};

std::unordered_map<uint64_t,Node> dag;

bool LimitedDFS(xyLoc const& start, xyLoc end, MapEnvironment const& env, int depth){
  if(depth==0)
	  return false;

  if(env.GoalTest(end,start)){

	  return true;
  }

  std::vector<xyLoc> successors;
  env.GetSuccessors(start,successors);
  for(auto const& node: successors){
	if(LimitedDFS(node,end,env,depth-1)){
	  dag[Node(node,depth).Hash()].parents.emplace_back(start,depth+1);
	  dag[Node(start,depth+1).Hash()].successors.emplace_back(node,depth);
	}
  }
}

int main(){
  MapEnvironment env(new Map(8,8));
  Node::env=env;
}
