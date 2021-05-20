#ifndef EPEADRIVER_H
#define EPEADRIVER_H
/*
 * $Id: sample.h,v 1.6 2006/09/18 06:23:39 nathanst Exp $
 *
 *  Driver.h
 *  hog
 *
 *  Created by Thayne Walker, Rajat Kumar Jenamani.
 *  Copyright (c) Thayne Walker, Rajat Kumar Jenamani 2021 All rights reserved.
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

#include "PEAStar.h"
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>

void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyComputationHandler();
void MyDisplayHandler(unsigned long windowID, tKeyboardModifier, char key);
void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier, char key);
void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier, char key);
int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
void InstallHandlers();
void InitSim();

#define INF 9999999.0f

template<typename T, typename C>
class custom_priority_queue : public std::priority_queue<T, std::vector<T>, C>
{
  public:

    /*bool remove(const T& value) {
     *       toDelete.insert(value->key());
     *           }
     *               bool removeAll(int agent, int count){
     *                     for(auto const& value:this->c){
     *                             if(value->sizes[agent]<count){
     *                                       toDelete.insert(value->key());
     *                                               }
     *                                                     }
     *                                                           return true;
     *                                                               }*/
    virtual T popTop(){
      T val(this->top());
      this->pop();
      /*while(toDelete.find(val->key()) != toDelete.end()){
       *         val=this->top();
       *                 this->pop();
       *                       }*/
      return val;
    }
};
#endif
