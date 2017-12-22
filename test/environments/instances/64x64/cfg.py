#!/usr/bin/python
import os

envs={4:'fiveconnected',
  8:'nineconnected',
  24:'twentyfiveconnected',
  48:'fortynineconnected'}

for d in [d for d in os.listdir(".") if os.path.isdir(os.path.join(".",d))]:
  for f in os.listdir(d):
    for b in [4,8,24,48]:
      content='''# Multi-Agent, Multi-Domain Problem file
# ==================================================
# Group Domain Environment Weight Start Stop Tether
# ==================================================
'''
      with open(os.path.join(d,f),"r") as fp:
        a=0
        for line in fp:
          s,g=line.split()
          content += "%d %d G %s 1.0 %s,0,0 %s,0,0 NONE\n"%(a,a,envs[b],s,g)
        a+=1

      #print content
      fn="%s.%d.cfg"%(f.split('.')[0],b)
      with open(os.path.join(d,fn),"w") as fp:
        fp.write(content)
    
    #with open("./%d/%d.csv"%(size,i), "w") as f:
      #for j in range(size):
        #f.write("%d,%d %d,%d\n"%(start[j][0],start[j][1],goal[j][0],goal[j][1]))
    
