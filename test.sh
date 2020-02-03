#!/bin/bash

ulimit -c 0

ALG="${ALG:-MACBS}"
BINDIR="${BINDIR:-/hog2/bin}"
#NAGENTS="${NAGENTS:-100}"
DOMAINTYPE="${DOMAINTYPE:-random}"
DOMAIN="${DOMAIN:-Berlin_1_256-@,2,1000,1:Boston_0_256-@,2,1000,1:Paris_1_256-@,2,1000,1:brc202d-@,2,1000,1:den312d-@,2,1000,1:den520d-@,2,1000,1:empty-16-16-@,2,128,1:empty-32-32-@,2,512,1:empty-48-48-@,2,1000,1:empty-8-8-@,2,32,,1:ht_chantry-@,2,1000,1:ht_mansion_n-@,2,1000,1:lak303d-@,2,1000,1:lt_gallowstemplar_n-@,2,1000,1:maze-128-128-10-@,2,1000,1:maze-128-128-2-@,2,1000,1:maze-32-32-2-@,2,333,1:maze-32-32-4-@,2,395,1:orz900d-@,2,1000,1:ost003d-@,2,1000,1:random-32-32-10-@,2,461,1:random-32-32-20-@,2,409,1:random-64-64-10-@,2,1000,1:random-64-64-20-@,2,1000,1:room-32-32-4-@,2,341,1:room-64-64-16-@,2,1000,1:room-64-64-8-@,2,1000,1:w_woundedcoast-@,2,1000,1}"
TRIALS="${TRIALS:-25}"
KILLTIME="${KILLTIME:-30}"
KILLMEM="${KILLMEM:-8000}"
BFACTOR="${BFACTOR:-9}"
RADIUS="${RADIUS:-.25}"
BYPASS="${BYPASS:-false}"
PC="${PC:-false}"
W="${W:-1}"
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
OVERLOAD="${OVERLOAD:-false}"
COMPLETE="${COMPLETE:-false}"
TOPTWO="${TOPTWO:-false}"
ID="${ID:-true}"
RES="${RES:-10}"
DISAPPEAR="${DISAPPEAR:-true}"
SUFFIX="${SUFFIX:-}"
APRIORI="${APRIORI:-false}"

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
8)
  DESCRIP="1xNTIME"
  ;;
9)
  DESCRIP="MxNTIME"
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

if [[ $APRIORI = true ]]; then
  APRIORI="-apriori"
  DESCRIP="${DESCRIP}_APRIORI"
else
  APRIORI=""
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

if [[ $OVERLOAD = true ]]; then
  OVERLOAD="-overload"
  DESCRIP="${DESCRIP}_OVERLOAD"
else
  OVERLOAD=""
fi

if [[ $COMPLETE = true ]]; then
  COMPLETE="-complete"
  DESCRIP="${DESCRIP}_COMPLETE"
else
  COMPLETE=""
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

DESCRIP="${DESCRIP}_W${W}"
DESCRIP="${DESCRIP}_${SUFFIX}"
DESCRIP="${DESCRIP}r${RADIUS}"

IFS=':' read -r -a DOMAIN <<< "$DOMAIN"
IFS=':' read -r -a BFACTOR <<< "$BFACTOR"

for d in "${!DOMAIN[@]}"; do
  dom=${DOMAIN[$d]}
  IFS=',' read -r -a dom <<< "$dom"
  dname=$(echo ${dom[0]} | sed "s/@/${DOMAINTYPE}/")
  START=${dom[1]}
  STOP=${dom[2]}
  INC=${dom[3]}

  for ((i=1; i<=${TRIALS}; i++)); do
    for b in "${!BFACTOR[@]}"; do
      MOV="-envfile /hog2/movement/env${BFACTOR[$b]}.csv"
      SCENARIOPREFIX="-probfile /hog2/instances/64x64/"
      SCENARIOSUFFIX=".csv -dimensions 64,64,1"
      if [[ ${dname} == "8" ]]; then
        SCENARIOPREFIX="-probfile /hog2/instances/8x8/50"
        SCENARIOSUFFIX=".csv -dimensions 8,8,1"
      elif [[ ${dname} == "64" ]]; then
        SCENARIOPREFIX="-probfile /hog2/instances/64x64/1000"
        SCENARIOSUFFIX=".csv -dimensions 64,64,1"
      else
        SCENARIOPREFIX=$( echo "-scenfile /hog2/benchmarks/scen-@" | sed "s/@/${DOMAINTYPE}/")
        SCENARIOSUFFIX=".scen"
      fi

      a=2
      last=512
      min=$a
      while [[ $last -ne $(( $min+1 )) ]]; do
      #for ((a=${START}; a<=${STOP}; a+=${INC})); do
        SCENPRIOR="-nagents ${a} "
        FILENAME=$(printf "${ALG}_${dname}_${DESCRIP}_%02d_%03d.txt" ${BFACTOR[$b]} ${i})
        if [[ ${ALG} == "icts" ]]; then
          MOV="-agentType ${BFACTOR[$b]}"
          cmd="${BINDIR}/${ALG} -w ${W} -seed ${a} -mode b -killtime $KILLTIME -killmem $KILLMEM ${MOV} ${SCENPRIOR}${SCENARIOPREFIX}/${dname}-${i}${SCENARIOSUFFIX} $ID $DISAPPEAR -timeRes $RES -radius $RADIUS -verify -quiet -nogui"
          echo $cmd $FILENAME
          stdbuf -oL $cmd >> /output/$FILENAME
        else
          cmd="${BINDIR}/${ALG} -seed ${a} -killtime $KILLTIME -killmem $KILLMEM ${SCENPRIOR}${MOV} ${SCENARIOPREFIX}/${dname}-${i}${SCENARIOSUFFIX} $BP $ASYM $VC $PC $CAT $XOR $CCT $COMPLETE $OVERLOAD $SUBOPT $TOPTWO $GREEDY $ID $DISAPPEAR -resolution $RES -ctype $CTYPE -radius $RADIUS -killmem $KILLMEM -verify -quiet -nogui -mergeThreshold 9999999 $APRIORI"
          echo "$cmd >> $FILENAME"
          stdbuf -oL $cmd >> /output/$FILENAME

          if [ $? -eq 0 ]; then
            min=$a
            if [[ $(( $a*2 )) -lt $last ]]; then
              a=$(( $a*2 ))
            else
              a=$(( ($a + $last)/2 ))
            fi
          else
            last=$a
            a=$(( ($min + $last)/2 ))
          fi
          if [[ $min -gt $a ]]; then
            a=$(( $min +1 ))
          fi
        fi
      done
    done
  done
done

