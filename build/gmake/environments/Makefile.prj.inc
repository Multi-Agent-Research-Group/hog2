#-----------------------------------------------------------------------------
# GNU Makefile for static libraries: project dependent part
#
# $Id: Makefile.prj.inc,v 1.2 2006/11/01 23:34:14 nathanst Exp $
# $Source: /usr/cvsroot/project_hog/build/gmake/graph/Makefile.prj.inc,v $
#-----------------------------------------------------------------------------

NAME = environments
DBG_NAME = lib$(NAME).a
REL_NAME = lib$(NAME).a
DBG_SO_NAME = lib$(NAME).so
REL_SO_NAME = lib$(NAME).so

ROOT = ../../..
VPATH = $(ROOT)

DBG_OBJDIR = $(ROOT)/objs/$(NAME)/debug
REL_OBJDIR = $(ROOT)/objs/$(NAME)/release
DBG_BINDIR = $(ROOT)/bin/debug
REL_BINDIR = $(ROOT)/bin/release

PROJ_CXXFLAGS = -I$(ROOT)/graph -I$(ROOT)/utils -I$(ROOT)/abstraction -I$(ROOT)/simulation -I$(ROOT)/graphalgorithms  -I$(ROOT)/generic -I$(ROOT)/algorithms -I$(ROOT)/search -I$(ROOT)/gui
PROJ_DBG_CXXFLAGS = $(PROJ_CXXFLAGS)
PROJ_REL_CXXFLAGS = $(PROJ_CXXFLAGS)

PROJ_DBG_LNFLAGS =
PROJ_REL_LNFLAGS =

PROJ_DBG_LIB =
PROJ_REL_LIB =

default : all

SRC_CPP = \
	environments/GraphEnvironment.cpp \
	environments/GraphRefinementEnvironment.cpp \
	environments/Grid3DConstrainedEnvironment.cpp \
	environments/Grid3DEnvironment.cpp \
	environments/GridStates.cpp \
	environments/Map2DEnvironment.cpp \
	environments/Map2DConstrainedEnvironment.cpp \
	environments/PermutationPuzzleEnvironment.cpp \
	environments/MNPuzzle.cpp \
	environments/FlipSide.cpp \
	environments/WeightedMap2DEnvironment.cpp \
	environments/Directional2DEnvironment.cpp \
	environments/TopSpin.cpp	\
	environments/RoboticArm.cpp  \
	environments/ConfigEnvironment.cpp \
	environments/PancakePuzzle.cpp \
	environments/Map3DGrid.cpp \
	environments/Map2DHeading.cpp \
	environments/MinimalSectorAbstraction.cpp \
	environments/MNAgentPuzzle.cpp \
	environments/RubiksCubeEdges.cpp \
	environments/RubiksCube7Edges.cpp \
	environments/RubiksCubeCorners.cpp \
	environments/RubiksCube.cpp \
	environments/CanonicalGrid.cpp \
	environments/Fling.cpp \
	environments/Voxels.cpp \
	environments/NaryTree.cpp 
	#environments/Airplane.cpp \
	#environments/AirplaneSimple.cpp \
	#environments/AirplaneGrid3DCardinal.cpp \
	#environments/AirplaneGrid3DOctile.cpp \
	#environments/AirplaneGridCardinal.cpp \
	#environments/AirplaneGridless.cpp \
	#environments/AirplaneHiFiGridless.cpp \
	#environments/AirplaneNaiveHiFiGridless.cpp \
	#environments/AirplaneGridOctile.cpp \
	#environments/AirplaneHighway.cpp \
	#environments/AirplaneHighway4.cpp \
	#environments/AirplaneHighway4Cardinal.cpp \
	#environments/AirplaneCardinal.cpp \
	#environments/AirplaneConstrained.cpp \
	#environments/AirStates.cpp \
