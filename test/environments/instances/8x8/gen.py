#!/usr/bin/python
import random
import os
import errno

for i in range(100):
  s=set()
  g=set()
  while len(s) < 20:
    s.add((random.randint(0,7),random.randint(0,7)))
  while len(g) < 20:
    g.add((random.randint(0,7),random.randint(0,7)))
  start=list(s)
  goal=list(g)
  
  for size in range(1,21):
    if not os.path.exists("./%d"%size):
      try:
        os.makedirs("./%d"%size)
      except OSError as exc:
        if exc.errno != errno.EEXIST:
          raise

    with open("./%d/%d.csv"%(size,i), "w") as f:
      for j in range(size):
        f.write("%d,%d %d,%d\n"%(start[j][0],start[j][1],goal[j][0],goal[j][1]))
    
