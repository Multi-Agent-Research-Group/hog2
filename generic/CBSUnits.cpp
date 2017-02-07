//
//  CBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "CBSUnits.h"

std::ostream& operator<<(std::ostream& ss, IntervalData v) {ss << "<H1:"<<v.hash1<<"-->"<<v.hash2<<" A:"<<unsigned(v.agent)<<">"; return ss;}


