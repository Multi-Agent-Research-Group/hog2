#!/bin/bash

ALG="${ALG:-MACBS}"
NAGENTS="${NAGENTS:-100}"
DOMAIN="${DOMAIN:-64}"
TRIALS="${TRIALS:-100}"
KILLTIME="${KILLTIME:-300}"
KILLMEM="${KILLMEM:-8000}"
BFACTOR="${BFACTOR:-9}"
RADIUS="${RADIUS:-.25}"
BYPASS="${BYPASS:-false}"
PC="${PC:-false}"
CAT="${CAT:-false}"
PRECHECK="${PRECHECK:-0}"
SKIP="${SKIP:-false}"
CCT="${CCT:-false}"
CTYPE="${CTYPE:-1}"
XOR="${XOR:-false}"
GREEDY="${GREEDY:-false}"
ID="${ID:-true}"
RES="${RES:-10}"
DISAPPEAR="${DISAPPEAR:-true}"

START=$NAGENTS
STOP=$NAGENTS
INC=1
if [[ $NAGENTS == *":"* ]]; then
  IFS=':' read -r -a NAGENTS <<< "$NAGENTS"
  START=${NAGENTS[0]}
  STOP=${NAGENTS[1]}
  INC=${NAGENTS[2]}
fi

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

BP=""
if [[ $BYPASS = false ]]; then
  BP="-nobypass"
else
  DESCRIP="${DESCRIP}_BP"
fi

IFS=':' read -r -a DOMAIN <<< "$DOMAIN"
IFS=':' read -r -a BFACTOR <<< "$BFACTOR"

for ((a=${START}; a<=${STOP}; a+=${INC})); do
  for d in "${!DOMAIN[@]}"; do
    SCENPRIOR=""
    SCENARIOPREFIX="-probfile /hog2/instances/64x64/"
    SCENARIOSUFFIX=".csv -dimensions 64,64,1"
    if [[ ${DOMAIN[$d]} == "8" ]]; then
      SCENARIOPREFIX="-probfile /hog2/instances/8x8/"
      SCENARIOSUFFIX=".csv -envfile /hog2/movement/env${BFACTOR[$b]}.csv -dimensions 8,8,1"
    elif [[ ${DOMAIN[$d]} == "64" ]]; then
      SCENARIOPREFIX="-probfile /hog2/instances/64x64/"
      SCENARIOSUFFIX=".csv -envfile /hog2/movement/env${BFACTOR[$b]}.csv -dimensions 64,64,1"
    else
      SCENPRIOR="-nagents ${a} "
      SCENARIOPREFIX="-scenfile /hog2/instances/${DOMAIN[$d]}/"
      SCENARIOSUFFIX=".csv -envfile /hog2/movement/env${BFACTOR[$b]}.csv"
    fi

    for b in "${!BFACTOR[@]}"; do
      FILENAME="cbs_${DOMAIN[$d]}_${BFACTOR[$b]}_${a}_${DESCRIP}.txt"
      for ((i=0; i<${TRIALS}; i++)); do
        cmd="/hog2/bin/${ALG} -seed ${i} -killtime $KILLTIME -killmem $KILLMEM ${SCENPRIOR}${SCENARIOPREFIX}${a}/${i}${SCENARIOSUFFIX} $BP $PC $CAT $XOR $CCT $GREEDY $ID $DISAPPEAR -resolution $RES -ctype $CTYPE -radius $RADIUS -killmem $KILLMEM -verify -quiet -nogui -mergeThreshold 9999999"
	time $cmd >> /output/$FILENAME
      done
    done
  done
done

