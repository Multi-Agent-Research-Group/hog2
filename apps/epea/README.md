# Extended Increasing Cost Tree Search for Non-Unit Cost Domains
https://www.ijcai.org/Proceedings/2018/0074.pdf

## To build and run the Extended EPEA algorithm
### Dependencies
* google test
* OpenGL (optional)

```
git clone -b id https://github.com/thaynewalker/hog2.git
cd hog2/build/gmake
```

## To build with graphics (for running when OpenGL is not installed)
```
make
```

## To build without graphics
```
make OPENGL=STUB
```

## To run EPEA

### For empty 8x8 grids
```
../../bin/release/epea -nagents 10 -agentType 9 -killtime 300 -killmem 16000 -mode p -probfile ../../test/environments/instances/8x8/50/0.csv -dimensions 8,8,1 -radius .25 -verify -nogui -disappear -quiet
```
### For benchmark DAO maps
```
../../bin/release/epea -nagents 20 -agentType 9 -killtime 300 -killmem 16000 -mode p -scenfile ../../test/environments/instances/den520d/200/0.csv -radius .25 -verify -nogui -disappear -quiet
```

### Meaning of flags 
* -killtime <secs> calls exit(0); with a timeout function which throws an error - you can safely ignore this error
* -killmem <mb> calls exit(0); if the resident memory size surpasses <mb> megabytes
* -nagents <n> selects the first n start/goal locations from -probfile <filename> (or from -scenfile for maps)
* -dimensions <n,n,n> should be passed after -probfile, it is not required for running with maps
* -radius <f> is the floating-point agent body radius
* -verify - checks for conflEPEA at the end of the run
* -disappear - agents disappear at the goal
* -quiet - turn off all output except final results
* -agentType: 4=4-connected, 5=4-connected+wait, 8=8-connected, 9=9-connected+wait, 24=16-connected, 25=16-connected+wait, 48=32-connected, 49=32-connected+wait,

### Output

cost is in 1/10 resolution (so you have to divide by ten)
```
seed:filepath,Connectedness,EPEANode::count,jointnodes,largestJoint,largestbranch,branchingfactor,Node::count,maxnagents,minsingle,maxsingle,minjoint,maxjoint,fullCollChecks,collChecks,total,mddTime,pairwiseTime,jointTime,nogoodTime,certifyTime,collTime,nacts,cost
6858:../../test/environments/instances/8x8/50/0.csv,9,1,9,1.11111,2,1.125,77,2,1,5,1,7,608,9,0.000514938,3.9497e-05,0,5.293e-05,0,6.9041e-05,0.000256746,22,190
```
terminate called without an active exception <-- This is normal  - it is from calling exit(0);

