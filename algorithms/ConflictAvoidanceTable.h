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
#ifndef _ConflictAvoidanceTable_h__
#define _ConflictAvoidanceTable_h__

#include <vector>
#include "SearchEnvironment.h"

// Stores paths for agents, indexed by time
template <typename BB, typename action>
class ConflictAvoidanceTable{
public:
  ConflictAvoidanceTable(){}
  virtual void set(std::vector<std::vector<typename BB::State>*> const*const ref){};
  virtual void remove(std::vector<typename BB::State> const& values, SearchEnvironment<typename BB::State,action> const*, unsigned agent)=0;
  virtual void insert(std::vector<typename BB::State> const& values, SearchEnvironment<typename BB::State,action> const*, unsigned agent)=0;
};

#endif
