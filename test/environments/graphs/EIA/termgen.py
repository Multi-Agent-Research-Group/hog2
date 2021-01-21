#!/usr/bin/python3

# load vertices
v={}
with open('vertices.txt','r') as vfp:
  for line in vfp:
    l=line.split(' ')
    v[int(l[0])]=[int(f) for f in l[1].split(',')]
print(v)

# load terminals
t=None
with open('terminals.txt','r') as vfp:
  t=[int(line) for line in vfp]
print(t)

def dist(a,b):
  import numpy as np
  return np.linalg.norm(np.array(a)-np.array(b))

for i in range(len(t)-1):
  for j in range(i+1,len(t)):
    print('dist(',t[i],t[j],dist(v[t[i]],v[t[j]]))

# break up terminal nodes into two extra nodes (incoming and outgoing)
# this will make it so that an agent cannot go into a destination and back out during planning
t2={}
for i in t:
  t2[i]=max(v)+1
  v[max(v)+1]=v[i]
print(t2)

# load edges (bidirectional)
e={}
with open('edges.txt','r') as vfp:
  for line in vfp:
    l=[int(a) for a in line.split(' ')[0].split(',')]
    if l[0] in t:
      if l[0] not in e:
        e[l[0]]=[l[1]]
      else:
        e[l[0]].append(l[1])

      print('instead of ',l[1],'-->',l[0],"do",l[1],'-->',t2[l[0]])
      if l[1] not in e:
        e[l[1]]=[t2[l[0]]] # replace destination with surrogate
      else:
        e[l[1]].append(t2[l[0]]) # replace destination with surrogate
      
    elif l[1] in t:
      print('instead of ',l[0],'-->',l[1],"do",l[0],'-->',t2[l[1]])
      if l[0] not in e:
        e[l[0]]=[t2[l[1]]] # replace destination with surrogate
      else:
        e[l[0]].append(t2[l[1]]) # replace destination with surrogate

      if l[1] not in e:
        e[l[1]]=[l[0]]
      else:
        e[l[1]].append(l[0])
    else:
      if l[0] not in e:
        e[l[0]]=[l[1]]
      else:
        e[l[0]].append(l[1])

      if l[1] not in e:
        e[l[1]]=[l[0]]
      else:
        e[l[1]].append(l[0])
print(e)

with open('dirvertices.txt','w') as fp:
  ids=[i for i in v.keys()]
  ids.sort()
  for i in ids:
    fp.write("%d %d,%d\n"%(i,v[i][0],v[i][1]))

with open('diredges.txt','w') as fp:
  ids=[i for i in e.keys()]
  ids.sort()
  for i in ids:
    for d in e[i]:
      fp.write("%d,%d\n"%(i,d))

with open('dirdest.txt','w') as fp:
  for i in t2.values():
    fp.write("%d\n"%i)

with open('dirstart.txt','w') as fp:
  for i in t:
    fp.write("%d\n"%i)
