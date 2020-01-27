# Generalized and Sub-optimal Constraints for Conflict-Based Search
(Will appear at AAAI 2020)

## To build and run the CBS algorithm with extensions for non-unit costs
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

## To run CBS
### 32-connected motion on 8x8 grid
```
../../bin/release/MACBS -nagents 10  -envfile env49.csv -killtime 300 -killmem 16000 -probfile ../../test/environments/instances/8x8/50/1.csv -dimensions 8,8,1 -radius .25 -mergeThreshold 9999999 -verify -nobypass -precheck 0 -disappear -quiet -nogui
```
### 32-connected motion on DAO benchmark maps
```
../../bin/release/MACBS -nagents 20  -envfile env49.csv -killtime 300 -killmem 16000 -scenfile ../../test/environments/instances/den520d/200/0.csv -radius .25 -mergeThreshold 9999999 -verify -nobypass -precheck 0 -disappear -quiet -nogui
```

### For help with flags
../../bin/release/MACBS -h

The 2^k neighborhood connectivities can be specified with -envfile <filename> (default is 4+wait)
These files are in the build/gmake directory:
env49.csv = 32-connected + wait
env48.csv = 32-connected
env25.csv = 16-connected + wait
env24.csv = 16-connected
env9.csv = 8-connected + wait
env8.csv = 8-connected
env5.csv = 4-connected + wait
env4.csv = 4-connected

* -mergeThreshold is the MA-CBS merge threshold, just set it to something really big for now -- I used to have ICTS working as a meta-agent solver, but it I haven't tested it for awhile
* -nobypass - Always use this for 2^k neighborhoods larger than 4-connected
* -nogui - omit this if you want to see the planner in action! However - it does not work in the ID framework, so to see graphics replace "-nogui" with "-noid" or "-noid -animate 1"

### Output
```
runtime and cost highlighted:
elapsed,planTime,replanTime,bypassplanTime,maplanTime,collisionTime,expansions,CATcollchecks,collchecks,collisions,cost,actions,meanCSet
VALID
7425:4.56221,0,4.55008,0,0,0.00113195,53148,0,5245,38,2982.6,1578,2
```
