#!/usr/bin/python3
import sys
import subprocess
import igraph as ig
def plotTree(edges,labels,highlight,name):
  mx=0
  for e in edges:
    mx=max(mx,e[1])
  print(mx,len(labels))
  g=ig.Graph(n=len(labels),directed=True)
  g.add_edges(edges)
  g.vs['label']=labels #["{}\n{}".format(r,labels[r]) for r in range(len(labels))]
  for i in range(len(labels)):
    if '*' in labels[i]:
      g.vs[i]['color']='#F99'
  g.vs["label_size"]=[12]*len(labels)
  g.vs["shape"]=['rectangle']*len(labels)
  g.vs["size"]=[80]*len(labels)
  for i in highlight:
    g.vs[i]['color']='gold' 
  #g.write_svg('searchtree%s.svg'%name,
  #layout=g.layout_reingold_tilford(root=[0]),
  #rescale=False,bbox=(32000,2000))
  ig.plot(g,
  margin=50,
  layout=g.layout_reingold_tilford(root=[0]),
  rescale=False,bbox=(2400,800))

labels=[]
process = subprocess.run('grep label %s'%sys.argv[1], shell=True, check=True, stdout=subprocess.PIPE, universal_newlines=True)
for line in process.stdout.split('\n'):
  try:
    labels.append(line.split('"')[1].replace("\\n","\n").replace('4294967295','inf'))
  except:
    pass

edges=[]
process = subprocess.run('grep edge %s'%sys.argv[1], shell=True, check=True, stdout=subprocess.PIPE, universal_newlines=True)
for line in process.stdout.split('\n'):
  try:
    edges.append(eval(line.split(' ')[1]))
  except:
    pass

path=[]
try:
  process = subprocess.run('grep -e "path [0-9]" %s'%sys.argv[1], shell=True, check=True, stdout=subprocess.PIPE, universal_newlines=True)
  path=[int(p) for p in process.stdout.split(' ')[1].split(',')]
except:
  pass

plotTree(edges,labels,path,sys.argv[1])
