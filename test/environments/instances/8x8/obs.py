#!/usr/bin/python
import random
import os
import errno
import sys

letters='abcdefghijklmnopqrstuvwxyz'
LETTERS='ABCDEFGHIJKLMNOPQRSTUVWXYZ'

def mapToStr(s,g,o,w=8,h=8):
  val=''
  for y in range(h):
    for x in range(w):
      if (x,y) in s:
        val+=letters[s.index((x,y))]
      elif (x,y) in g:
        val+=LETTERS[g.index((x,y))]
      elif (x,y) in o:
        val += '@'
      else:
        val += '.'
    val+="\n"
  return val

def expand(n,o,v,w=8):
  #print "expand",n
  s=[]
  x1=(n[0]+1,n[1])
  x2=(n[0]-1,n[1])
  x3=(n[0],n[1]+1)
  x4=(n[0],n[1]-1)
  if x1[0]<w and x1 not in o and x1 not in v:
    s.append(x1)
  if x2[0]>-1 and x2 not in o and x2 not in v:
    s.append(x2)
  if x3[1]<w and x3 not in o and x3 not in v:
    s.append(x3)
  if x4[1]>-1 and x4 not in o and x4 not in v:
    s.append(x4)
  #print "generated",s
  return s

def dfs(s,g,o,v):
  v.add(s)
  if s==g:
    return True
  else:
    for n in expand(s,o,v):
      if dfs(n,g,o,v):
        return True
  return False

def solvable(s,g,o):
  for S,G in zip(s,g):
    if not dfs(S,G,o,set()):
      return False
  return True

def getObstacles(s,g,n=13):
  while True:
    o=set()
    while len(o) < n:
      obs=(random.randint(0,7),random.randint(0,7))
      if obs not in s and obs not in g:
        o.add(obs)
    if solvable(s,g,o): break

  return list(o)

for i in range(100):
  s=set()
  g=set()
  while len(s) < 20:
    s.add((random.randint(0,7),random.randint(0,7)))
  while len(g) < 20:
    g.add((random.randint(0,7),random.randint(0,7)))
  start=list(s)
  goal=list(g)
  obs=getObstacles(start,goal)
  
  for size in range(1,21):
    if not os.path.exists("./%d"%size):
      try:
        os.makedirs("./%d"%size)
      except OSError as exc:
        if exc.errno != errno.EEXIST:
          raise

    with open("./%d/%d.map"%(size,i), "w") as f:
      for j in range(size):
        f.write("type octile\nheight 8\nwidth 8\nmap\n")
        f.write(mapToStr([],[],obs))

    with open("./%d/%d.map.scen"%(size,i), "w") as f:
      for j in range(size):
        f.write("version 1\n")
        f.write("%d\ttest/environments/instances/8x8/%d/%d.map\t8\t8\t%d\t%d\t%d\t%d\t0.0\n"%(j,size,i,start[j][0],start[j][1],goal[j][0],goal[j][1]))
    
