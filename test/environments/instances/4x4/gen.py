#!/usr/bin/python
import random
import os
import errno

for i in range(100):
  s=set()
  g=set()
  while len(s) < 15:
    s.add((random.randint(0,3),random.randint(0,3)))
  while len(g) < 15:
    g.add((random.randint(0,3),random.randint(0,3)))
  start=list(s)
  goal=list(g)
  
  for size in range(2,16):
    if not os.path.exists("./%d"%size):
      try:
        os.makedirs("./%d"%size)
      except OSError as exc:
        if exc.errno != errno.EEXIST:
          raise

    with open("./%d/%d.csv"%(size,i), "w") as f:
      for j in range(size):
        f.write("%d,%d %d,%d\n"%(start[j][0],start[j][1],goal[j][0],goal[j][1]))
    
