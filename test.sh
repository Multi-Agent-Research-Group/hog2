#!/bin/bash

ulimit -c 0

ALG="${ALG:-MACBS}"
#NAGENTS="${NAGENTS:-100}"
DOMAIN="${DOMAIN:-8,5,35,5:64,20,260,20:brc202d,10,100,10:ost003d,10,100,10:den520d,10,100,10}"
TRIALS="${TRIALS:-100}"
KILLTIME="${KILLTIME:-300}"
KILLMEM="${KILLMEM:-8000}"
BFACTOR="${BFACTOR:-9}"
RADIUS="${RADIUS:-.25}"
BYPASS="${BYPASS:-false}"
PC="${PC:-false}"
VC="${VC:-false}"
ASYM="${ASYM:-false}"
CAT="${CAT:-false}"
PRECHECK="${PRECHECK:-0}"
SKIP="${SKIP:-false}"
CCT="${CCT:-false}"
CTYPE="${CTYPE:-1}"
XOR="${XOR:-false}"
GREEDY="${GREEDY:-false}"
SUBOPT="${SUBOPT:-false}"
CONDITIONAL="${CONDITIONAL:-false}"
TOPTWO="${TOPTWO:-false}"
ID="${ID:-true}"
RES="${RES:-10}"
DISAPPEAR="${DISAPPEAR:-true}"

DESCRIP=""
case $CTYPE in
2)
  DESCRIP="PYR"
  ;;
3)
  DESCRIP="CONF"
  ;;
4)
  DESCRIP="TIME"
  ;;
5)
  DESCRIP="MC"
  ;;
6)
  DESCRIP="OVR"
  ;;
7)
  DESCRIP="BOX"
  ;;
*)
  DESCRIP="IDENT"
  ;;
esac

case $PRECHECK in
1)
  DESCRIP="${DESCRIP}_BB"
  ;;
2)
  DESCRIP="${DESCRIP}_CH"
  ;;
*)
  DESCRIP="${DESCRIP}_NOPRE"
  ;;
esac

if [[ $SKIP = true ]]; then
  SKIP="-skip"
  DESCRIP="${DESCRIP}_SKIP"
else
  SKIP=""
fi

DESCRIP="${DESCRIP}_RES${RES}"

if [[ $DISAPPEAR = true ]]; then
  DISAPPEAR="-disappear"
  DESCRIP="${DESCRIP}_DIS"
else
  DISAPPEAR=""
fi

if [[ $ID = false ]]; then
  ID="-noid"
else
  DESCRIP="${DESCRIP}_ID"
  ID=""
fi

if [[ $GREEDY = true ]]; then
  GREEDY="-greedyCT"
  DESCRIP="${DESCRIP}_GREEDY"
else
  GREEDY=""
fi

if [[ $CONDITIONAL = true ]]; then
  CONDITIONAL="-conditional"
  DESCRIP="${DESCRIP}_COND"
else
  CONDITIONAL=""
fi

if [[ $SUBOPT = true ]]; then
  SUBOPT="-suboptimal"
  DESCRIP="${DESCRIP}_SUBOPT"
else
  SUBOPT=""
fi

if [[ $TOPTWO = true ]]; then
  TOPTWO="-toptwo"
  DESCRIP="${DESCRIP}_TOPTWO"
else
  TOPTWO=""
fi

if [[ $XOR = true ]]; then
  XOR="-xor"
  DESCRIP="${DESCRIP}_XOR"
else
  XOR=""
fi

if [[ $CCT = true ]]; then
  CCT="-cct"
  DESCRIP="${DESCRIP}_CCT"
else
  CCT=""
fi

if [[ $CAT = true ]]; then
  CAT="-cat"
  DESCRIP="${DESCRIP}_CAT"
else
  CAT=""
fi

if [[ $PC = true ]]; then
  PC="-pc"
  DESCRIP="${DESCRIP}_PC"
else
  PC=""
fi

if [[ $ASYM = true ]]; then
  ASYM="-asym"
  DESCRIP="${DESCRIP}_ASYM"
else
  ASYM=""
fi

if [[ $VC = true ]]; then
  VC="-vc"
  DESCRIP="${DESCRIP}_VC"
else
  VC=""
fi

BP=""
if [[ $BYPASS = false ]]; then
  BP="-nobypass"
else
  DESCRIP="${DESCRIP}_BP"
fi

DESCRIP="${DESCRIP}r${RADIUS}"

IFS=':' read -r -a DOMAIN <<< "$DOMAIN"
IFS=':' read -r -a BFACTOR <<< "$BFACTOR"

for d in "${!DOMAIN[@]}"; do
dom=${DOMAIN[$d]}
IFS=',' read -r -a dom <<< "$dom"
dname=${dom[0]}
START=${dom[1]}
STOP=${dom[2]}
INC=${dom[3]}

for ((a=${START}; a<=${STOP}; a+=${INC})); do
  for b in "${!BFACTOR[@]}"; do
      MOV="-envfile /hog2/movement/env${BFACTOR[$b]}.csv"
      SCENPRIOR="-nagents ${a} "
      SCENARIOPREFIX="-probfile /hog2/instances/64x64/"
      SCENARIOSUFFIX=".csv -dimensions 64,64,1"
      if [[ ${dname} == "8" ]]; then
        SCENARIOPREFIX="-probfile /hog2/instances/8x8/50"
        SCENARIOSUFFIX=".csv -dimensions 8,8,1"
      elif [[ ${dname} == "64" ]]; then
        SCENARIOPREFIX="-probfile /hog2/instances/64x64/1000"
        SCENARIOSUFFIX=".csv -dimensions 64,64,1"
      else
        SCENARIOPREFIX="-scenfile /hog2/instances/${dname}/500"
        SCENARIOSUFFIX=".csv"
      fi

      FILENAME=$(printf "${ALG}_${dname}_${DESCRIP}_%02d_%03d.txt" ${BFACTOR[$b]} ${a})
      for ((i=0; i<${TRIALS}; i++)); do
        if [[ ${ALG} == "icts" ]]; then
          MOV="-agentType ${BFACTOR[$b]}"
          cmd="/hog2/bin/${ALG} -seed ${i} -mode p -killtime $KILLTIME -killmem $KILLMEM ${MOV} ${SCENPRIOR}${SCENARIOPREFIX}/${i}${SCENARIOSUFFIX} $ID $DISAPPEAR -timeRes $RES -radius $RADIUS -verify -quiet -nogui"
          echo $cmd $FILENAME
          time $cmd >> /output/$FILENAME
        else
          cmd="/hog2/bin/${ALG} -seed ${i} -killtime $KILLTIME -killmem $KILLMEM ${SCENPRIOR}${MOV} ${SCENARIOPREFIX}/${i}${SCENARIOSUFFIX} $BP $ASYM $VC $PC $CAT $XOR $CCT $CONDITIONAL $SUBOPT $TOPTWO $GREEDY $ID $DISAPPEAR -resolution $RES -ctype $CTYPE -radius $RADIUS -killmem $KILLMEM -verify -quiet -nogui -mergeThreshold 9999999"
          echo $cmd $FILENAME
          time $cmd >> /output/$FILENAME
        fi
      done
    done
  done
done

