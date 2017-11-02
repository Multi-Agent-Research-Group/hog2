#!/usr/bin/python
import random
import os
import errno
import sys

mapname=sys.argv[1]

h=0
w=0
m=[]
with open(mapname,"r") as fp:
  t=fp.readline()
  h=int(fp.readline().split()[1])
  w=int(fp.readline().split()[1])
  t=fp.readline()
  m=fp.readlines()

print h
print w
h=len(m)
w=len(m[0])
print h
print w


for i in range(100):
  s=set()
  g=set()
  while len(s) < 300:
    x=random.randint(0,w-1)
    y=random.randint(0,h-1)
    while m[y][x]!='.':
      x=random.randint(0,w-1)
      y=random.randint(0,h-1)
    s.add((x,y))
  while len(g) < 300:
    x=random.randint(0,w-1)
    y=random.randint(0,h-1)
    while m[y][x]!='.':
      x=random.randint(0,w-1)
      y=random.randint(0,h-1)
    g.add((x,y))
  start=list(s)
  goal=list(g)
  
  for size in range(5,300,5):
    if not os.path.exists("./%d"%size):
      try:
        os.makedirs("./%d"%size)
      except OSError as exc:
        if exc.errno != errno.EEXIST:
          raise

    with open("./%d/%d.csv"%(size,i), "w") as f:
      f.write("version 1\n")
      for j in range(size):
        f.write("0\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t0\n"%(mapname[mapname.index('maps'):],w,h,start[j][0],start[j][1],goal[j][0],goal[j][1]))
