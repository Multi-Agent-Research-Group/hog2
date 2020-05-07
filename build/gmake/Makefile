#-----------------------------------------------------------------------------
# Main Makefile for project_hog.
# The sub-makefiles are invoked sequentially
# (see: Peter Miller: Recursive Make Considered Harmful
# http://aegis.sourceforge.net/auug97.pdf)
#
# $Source: /usr/cvsroot/project_hog/build/gmake/Makefile,v $
# $Id: Makefile,v 1.5 2006/11/01 23:34:14 nathanst Exp $
#-----------------------------------------------------------------------------

# All projects to build in this order.
PROJECTS = \
  gui/GL \
  absmapalgorithms \
  abstraction \
  abstractionalgorithms \
  algorithms \
  environments \
  generic \
  graph \
  grids \
  graphalgorithms \
  gui \
  mapalgorithms \
  shared \
  utils \
  apps/icts \
  apps/MACBS \
  apps/CBICS \
  apps/NSF \
  apps/MAAStar \
  test/utils \
  #test/collisiondetection \
  #search \
  #apps/experimental \
  #apps/mapUtils \
  #apps/3dmaps \
  #apps/MAIDAStar \
  #test/mapf \
  #test/environments \
  #apps/MAPF \
  #apps/aacbs \
  #apps/airplane \
  #test/airplane \
  #apps/MO \
#  apps/multiagent \
#  apps/measure_inconsistency 
#  simulation \
#  learning \
#	apps/coprobber
#  apps/lv.pathfinding	
#  apps/inconsistency \




# Include projects with broken compilation
PROJECTS_ALL = \
  gui/GL \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/idastar \
  apps/rubik \
  apps/bidirectional \
  apps/voxel \
  apps/fling \
  apps/topspin \
  apps/stp \
  apps/pancake \
  apps/roads \
  apps/canonicalGrids \
  apps/delta \
  apps/prediction \
  apps/emotion \
  apps/sfbds \
  apps/arm \
  apps/localsensingsearch \
  apps/cpdb \
  apps/pathsample \
  apps/puzzlesample \
  apps/inconsistency \
  apps/dynamicsearch \
  apps/directional \
  apps/dragonage \
  absmapalgorithms \
  abstraction \
  abstractionalgorithms \
  algorithms \
  environments \
  generic \
  graph \
  grids \
  graphalgorithms \
  gui \
  mapalgorithms \
  shared \
  simulation \
  utils \
  apps/arm \
  apps/airplane \
  apps/localsensingsearch \
  apps/cpdb \
  apps/pathsample \
  apps/puzzlesample \
  apps/inconsistency \
  apps/multiagent \
  apps/dynamicsearch \
  apps/directional \
  apps/mapUtils \
  apps/dragonage \
  apps/sfbds \
  apps/prediction \
  apps/rubik \
  apps/fling \
  apps/bidirectional \
  apps/voxel \
  apps/topspin \
  apps/stp \
  apps/roads \
  apps/delta \
  apps/pancake \
  apps/multiagent \
  demos/DFID \
  demos/dijkstra \
  demos/astar \
  demos/idastar \
  test/airplane \
  test/collisiondetection \
  test/environments \
#	apps/coprobber

# sequentially to avoid same sub-target in sub-make invoked twice
default:
	$(MAKE) debug
	$(MAKE) release

debug:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (dbg)"; \
             $(MAKE) -C $$dir debug || exit -1; \
           fi; \
        done
	@echo ""

release:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- $$dir (rel)"; \
             $(MAKE) -C $$dir release || exit -1; \
           fi; \
        done
	@echo ""

clean:
	@for dir in $(PROJECTS); do \
           if test -f $$dir/Makefile; then \
             echo -e "\n--------------- Removing temporary files for $$dir"; \
             $(MAKE) -C $$dir clean || exit -1; \
           fi; \
         done
	@echo ""

#check: debug
#check:
	#@for dir in $(PROJECTS); do \
           #if test -f $$dir/test/Makefile; then \
             #echo -e "\n--------------- $$dir (test)"; \
             #$(MAKE) -C $$dir/test debug || exit -1; \
           #fi; \
        #done
	#@echo ""


# sequentially to avoid same sub-target in sub-make invoked twice
all: override PROJECTS = $(PROJECTS_ALL)
all:
	$(MAKE) all-debug
	$(MAKE) all-release

all-debug: override PROJECTS = $(PROJECTS_ALL)
all-debug: debug

all-release: override PROJECTS = $(PROJECTS_ALL)
all-release: release

tags:
	@echo -e "Generating tags for Emacs"
	@(ctags -e `find ../.. -name "*.h" -o -name "*.cpp"`)

.PHONY: clean debug default release tags
