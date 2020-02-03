#!/usr/bin/python
#
#  Created by Thayne Walker.
#  Copyright (c) Thayne Walker 2017 All rights reserved.
#
# This file is part of HOG2.
#
# HOG2 is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# HOG2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with HOG; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
import random
import os
import errno

for i in range(100):
  s=set()
  g=set()
  while len(s) < 300:
    s.add((random.randint(0,63),random.randint(0,63)))
  while len(g) < 300:
    g.add((random.randint(0,63),random.randint(0,63)))
  start=list(s)
  goal=list(g)
  
  for size in range(5,305,5):
    if not os.path.exists("./%d"%size):
      try:
        os.makedirs("./%d"%size)
      except OSError as exc:
        if exc.errno != errno.EEXIST:
          raise

    with open("./%d/%d.csv"%(size,i), "w") as f:
      for j in range(size):
        f.write("%d,%d %d,%d\n"%(start[j][0],start[j][1],goal[j][0],goal[j][1]))
    
