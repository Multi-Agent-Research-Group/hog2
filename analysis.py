#!/usr/bin/python3

import numpy as np
import glob
import sys, os
import matplotlib.pyplot as plt
import pprint 
pp = pprint.PrettyPrinter(indent=2)

def autolabel(rects,ax,err):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect,sd in zip(rects,err):
        height = rect.get_height()
        ax.annotate('%.1f+/-%.1f'%(height,sd),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')

width=0.35
title=sys.argv[2]
f = glob.glob(sys.argv[1]+'/output.*.cb*.txt')
f.sort()
tests={'5':{},'9':{},'x':{},'25':{}}
#tests={9:{}}
for ff in f:
  c=ff.split(".")
  s=c[5]
  #if 'p' not in s or 'n' in s:
  #if not 'm' in s:
  #if 'p' in s or '1' in s:
  #if 'n' in s:
  #if '1' in s:
    #continue
  bf=c[2]
  n=c[4]
  if bf not in tests:
    continue
  if n not in tests[bf]:
    tests[bf][n]={}
  tests[bf][n][s]={}
  times=[]
  expanded=[]
  generated=[]
  with open(ff,'r') as fp:
    print(ff)
    for line in fp:
      if ":" in line and "," in line:
        l=line.split(':')
        trial=int(l[0])
        if len(times)<trial:
          times.append([])
          expanded.append([])
          generated.append([])
        stats=l[1].split(',')
        times[-1].append(float(stats[0]))
        expanded[-1].append(float(stats[1]))
        generated[-1].append(float(stats[2]))
  # pad the arrays
  maxlen=max([len(z) for z in times])
  Z=np.ones((len(times),maxlen))*30.0
  for i, row in enumerate(times):
    Z[i,:len(row)]=row
  tests[bf][n][s]['times']=np.mean(Z,axis=0)
  tests[bf][n][s]['timesd']=np.std(Z,axis=0)
  tests[bf][n][s]['success']=np.sum(Z < 30.0,axis=0)/len(Z)
  Z=np.zeros((len(expanded),maxlen))
  for i, row in enumerate(expanded):
    Z[i,:len(row)]=row
  tests[bf][n][s]['expanded']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))
  tests[bf][n][s]['expsd']=np.nan_to_num(np.nanstd(np.where(np.isclose(Z,0),np.nan, Z),axis=0))
  Z=np.zeros((len(generated),maxlen))
  for i, row in enumerate(generated):
    Z[i,:len(row)]=row
  tests[bf][n][s]['generated']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))

f = glob.glob(sys.argv[1]+'/output.*.icts.*.txt')
for ff in f:
  c=ff.split(".")
  s=c[5]
  #if 'p' not in s or 'n' in s:
  #if not 'm' in s:
  #if 'p' in s or '1' in s:
  #if 'n' in s:
  #if '1' in s:
    #continue
  bf=c[2]
  n=c[4]
  if bf not in tests:
    continue
  if n not in tests[bf]:
    tests[bf][n]={}
  tests[bf][n][s]={}
  times=[]
  expanded=[]
  generated=[]
  with open(ff,'r') as fp:
    print(ff)
    for line in fp:
      if ":" in line and "," in line:
        l=line.split(':')
        try:
          trial=int(l[0])
        except:
          continue
        if len(times)<trial:
          times.append([])
          expanded.append([])
          generated.append([])
        stats=l[1].split(',')
        times[-1].append(float(stats[0]))
        expanded[-1].append(float(stats[3]))
        generated[-1].append(float(stats[3]))
  # pad the arrays
  maxlen=max([len(z) for z in times])
  Z=np.ones((len(times),maxlen))*30.0
  for i, row in enumerate(times):
    Z[i,:len(row)]=row
  tests[bf][n][s]['times']=np.mean(Z,axis=0)
  tests[bf][n][s]['timesd']=np.std(Z,axis=0)
  tests[bf][n][s]['success']=np.sum(Z < 30.0,axis=0)/len(Z)
  Z=np.zeros((len(expanded),maxlen))
  for i, row in enumerate(expanded):
    Z[i,:len(row)]=row
  tests[bf][n][s]['expanded']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))
  tests[bf][n][s]['expsd']=np.nan_to_num(np.nanstd(np.where(np.isclose(Z,0),np.nan, Z),axis=0))
  Z=np.zeros((len(generated),maxlen))
  for i, row in enumerate(generated):
    Z[i,:len(row)]=row
  tests[bf][n][s]['generated']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))

    
f = glob.glob(sys.argv[1]+'/output.*.li.*.txt')
f.sort()
for ff in f:
  c=ff.split(".")
  s=c[5]
  if 'p' in s:
    continue
  bf=c[2]
  n=c[4]
  if bf not in tests:
    continue
  if n not in tests[bf]:
    tests[bf][n]={}
  tests[bf][n][s]={}
  times=[]
  expanded=[]
  with open(ff,'r') as fp:
    for line in fp:
      if ":" in line:
        l=line.split(':')
        trial=int(l[0])
        if len(times)<trial:
          times.append([])
          expanded.append([])
      if "," in line and " " not in line:
        stats=line.split(',')
        times[-1].append(float(stats[1]))
        expanded[-1].append(float(stats[2]))
  # pad the arrays
  maxlen=max([len(z) for z in times])
  Z=np.ones((len(times),maxlen))*30.0
  for i, row in enumerate(times):
    Z[i,:len(row)]=row
  tests[bf][n][s]['times']=np.mean(Z,axis=0)
  tests[bf][n][s]['timesd']=np.std(Z,axis=0)
  tests[bf][n][s]['success']=np.sum(Z < 30.0,axis=0)/10
  Z=np.zeros((len(expanded),maxlen))
  for i, row in enumerate(expanded):
    Z[i,:len(row)]=row
  tests[bf][n][s]['expanded']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))
  tests[bf][n][s]['expsd']=np.nan_to_num(np.nanstd(np.where(np.isclose(Z,0),np.nan, Z),axis=0))
  Z=np.zeros((len(generated),maxlen))
  #for i, row in enumerate(generated):
    #Z[i,:len(row)]=row
  #tests[bf][n][s]['generated']=np.nan_to_num(np.true_divide(np.sum(Z,axis=0),np.sum(Z !=0,axis=0)))

#pp.pprint(tests)


#print([k for k in tests[5].keys()])
#tests2={}
#for bf,b in tests.items():
  #tests2[bf]={k:{k2:v} for k2,v2 in b.items() for k,v in v2.items()}
#tests=tests2
#print([k for k in tests[5].keys()])

for bf,b in tests.items():
  print(bf)
  groups=list(tests[bf][list(tests[bf].keys())[0]].keys())

  for n,d in b.items():
    print(bf,n)
    print("================")
    fig, (ax1, ax2, ax3) = plt.subplots(1,3)
    fig.set_size_inches(20,12)
    fig.suptitle(n+" - bf={}".format(bf))
    ax1.set_title('#Nodes Expanded')
    ax2.set_title('Runtime')
    ax3.set_title('Success Rate')

    for i,s in enumerate(d):
      print(s)
      foo=np.array([(p,q) for p,q in zip(np.arange(len(d[s]['expanded']))+2,d[s]['expanded']) if p%5==0])
      ax1.plot(np.arange(len(d[s]['expanded']))+2,d[s]['expanded'],label=s,linewidth=10-i)#,yerr=d[s]['expsd'])
      foo3=np.array([(p,round(q*100)/100) for p,q in zip(np.arange(len(d[s]['times']))+2,d[s]['times']) if p%5==0])
      ax2.plot(np.arange(len(d[s]['times']))+2,d[s]['times'],label=s,linewidth=10-i)#,yerr=d[s]['timesd'])
      foo2=[str((p,round(q*100))) for p,q in zip(np.arange(len(d[s]['success']))+2,d[s]['success']) if p%5==0]
      print(' '.join([str(i[1]) for i in (foo3)][1:]))
      #print(' '.join([str(i[1]) for i in (foo)]))
      #print(' '.join([str(round(i[1]*1000,3)) for i in (foo3/foo)]))
      #print(' '.join([str(round(i[1]*1000,3)) for i in (foo/foo3)]))
      #print(' '.join(foo2))
      #print(' '.join([f.split()[1][:-1] for f in foo2]))
      ax3.plot(np.arange(len(d[s]['success']))+2,d[s]['success'],label=s,linewidth=10-i)
    ax3.legend(bbox_to_anchor=(1.1, 1.05),fontsize=16)
    #ax1.set_xticklabels(groups,fontsize=16)
    #ax2.set_xticklabels(groups,fontsize=16)
    #ax3.set_xticklabels(groups,fontsize=16)
    #ax1.set_yscale('log')
    #autolabel(plot,ax1,sd)


  fig.tight_layout()
  plt.show()

  #plt.subplots_adjust(left=0,right=1,bottom=0,top=1,wspace=1,hspace=1)
  #fig.savefig('origalgo-{}-{}.png'.format(title,bf))
  #plt.close(fig)

  
