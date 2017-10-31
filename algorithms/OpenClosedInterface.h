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

#ifndef hog2_glut_OpenClosedInterface_h
#define hog2_glut_OpenClosedInterface_h

enum dataLocation {
	kOpenList,
	kClosedList,
	kNotFound
};

const uint64_t kTAStarNoNode = 0xFFFFFFFFFFFFFFFFull;
const uint64_t kTAStarSelf = 0xFFFFFFFFFFFFFFFEull;

template<typename state>
class AStarOpenClosedData {
public:
	AStarOpenClosedData() {}
	AStarOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, uint64_t openLoc, dataLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), openLocation(openLoc), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	uint64_t openLocation;
	bool reopened;
	dataLocation where;
};


template<typename state, class dataStructure = AStarOpenClosedData<state> >
class OpenClosedInterface {
public:
	virtual uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode)=0;
	virtual uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTAStarNoNode)=0;
	virtual void KeyChanged(uint64_t objKey)=0;
	virtual dataLocation Lookup(uint64_t hashKey, uint64_t &objKey) const=0;
	virtual dataStructure& Lookup(uint64_t objKey)=0;
	virtual const dataStructure &Lookat(uint64_t objKey) const=0;
	virtual uint64_t Peek() const=0;
	virtual uint64_t Close()=0;
	virtual void Reopen(uint64_t objKey)=0;
	
	virtual uint64_t GetOpenItem(unsigned int which)=0;
	virtual size_t OpenSize() const=0;
	virtual size_t ClosedSize() const=0;
	virtual size_t size() const=0;
private:
};

#endif
