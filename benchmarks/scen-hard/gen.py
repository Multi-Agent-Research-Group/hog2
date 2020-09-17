import random
import sys

x=[]
with open(sys.argv[1],'r') as f:
  for line in f.readlines():
    if '.' in line or '@' in line:
      x.append(line.strip())
      print(line)

starts=[]
goals=[]
for i in range(len(x)):
  for j in range(len(x[0])):
    if x[i][j]=='.':
      starts.append((i,j))
      goals.append((i,j))
print(len(starts),"open spots")

with open(sys.argv[2],'w') as f:
  f.write('version 1\n')
  while len(starts) > 1:
    s=random.randint(0,len(starts)-1)
    e=random.randint(0,len(goals)-1)
    ss=starts[s]
    ee=goals[e]
    del starts[s]
    del goals[e]
    f.write('1\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t0.0\n'.format(sys.argv[1],len(x),len(x[0]),ss[1],ss[0],ee[1],ee[0]))
