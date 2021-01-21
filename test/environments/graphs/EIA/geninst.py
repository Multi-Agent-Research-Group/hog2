#!/usr/bin/python3
import random

s=[]
with open('dirstart.txt','r') as fp:
  for line in fp:
    s.append(int(line))

g=[]
with open('dirdest.txt','r') as fp:
  for line in fp:
    g.append(int(line))

v={}
with open('dirvertices.txt','r') as vfp:
  for line in vfp:
    l=line.split(' ')
    v[int(l[0])]=[int(f) for f in l[1].split(',')]

def dist(a,b):
  import numpy as np
  return np.linalg.norm(np.array(a)-np.array(b))

def vdist(a,b):
  return dist(v[a],v[b])

def pick(l):
  return l[random.randint(0,len(l)-1)]

def pickneither(k,l):
  while True:
    p=pick([vv for vv in v.keys()])
    if p not in k and p not in l:
      return p

def picknot(l):
  while True:
    p=pick([vv for vv in v.keys()])
    if p not in l:
      return p

radius=35

# random to/from terminal
for num in range(1,26):
  print('snapshot',num)
  inst=[]
  while len(inst)<30:
    if random.randint(0,1):
      #start at terminal
      while True:
        st=pick(s)
        ok=True
        for ss in inst:
          if vdist(st,ss[0]) <= radius:
             ok=False
             break
        if ok:
          break

      while True:
        en=picknot(s)
        ok=True
        for ss in inst:
          if vdist(en,ss[1]) <= radius:
             ok=False
             break
        if ok:
          break
      
      print('add',st,en)
      inst.append((st,en))
    else:
      #start somewhere else
      while True:
        st=picknot(g)
        ok=True
        for ss in inst:
          if vdist(st,ss[0]) <= radius:
             ok=False
             break
        if ok:
          break

      while True:
        en=pick(g)
        ok=True
        for ss in inst:
          if vdist(en,ss[1]) <= radius:
             ok=False
             break
        if ok:
          break
      
      print('add',st,en)
      inst.append((st,en))

  # check
  for i in range(len(inst)-1):
    for j in range(i+1,len(inst)):
      assert(vdist(inst[i][0],inst[j][0])>radius)
      assert(vdist(inst[i][1],inst[j][1])>radius)
  with open("snapshot-%d"%num,'w') as fp:
    for i in inst:
      fp.write("%d %d\n"%i)

# terminal to terminal
for num in range(1,26):
  print('term2term',num)
  inst=[]
  while len(inst)<20:
      #start at terminal
      while True:
        st=pick(s)
        ok=True
        for ss in inst:
          if vdist(st,ss[0]) <= radius:
             ok=False
             break
        if ok:
          break

      while True:
        en=pick(g)
        ok=True
        for ss in inst:
          if vdist(en,ss[1]) <= radius:
             ok=False
             break
        if ok:
          break
      
      print('add',st,en)
      inst.append((st,en))
  # check
  for i in range(len(inst)-1):
    for j in range(i+1,len(inst)):
      assert(vdist(inst[i][0],inst[j][0])>radius)
      assert(vdist(inst[i][1],inst[j][1])>radius)
  with open("term2term-%d"%num,'w') as fp:
    for i in inst:
      fp.write("%d %d\n"%i)
