#!/bin/bash
vys=( [-20.,0.,0.,-1.] [-19.,0.,0.,-1.] [-18,0.,0.,-1.] [-17.,0.,0.,-1.] [-16.,0.,0.,-1.] [-15.,0.,0.,-1.] [-14.,0.,0.,-1.] [-13.,0.,0.,-1.] [-12.,0.,0.,-1.] [-11,0.,0.,-1.] [-10.,0.,0.,-1.] [-9,0.,0.,-1.] [-8.,0.,0.,-1.] [-7,0.,0.,-1.] [-6.,0.,0.,-1.] [-5,0.,0.,-1.] [-4.,0.,0.,-1.] [-3,0.,0.,-1.] [-2.,0.,0.,-1.] [-1,0.,0.,-1.] [0.,0.,0.,-1.])
radi=( [9.,10.,5.,12.] [8.,10.,5.,12.] [7.,10.,5.,12.] [6.,10.,5.,12.] [5.,10.,5.,12.]  [4.,10.,5.,12.]  [3.,10.,5.,12.]  [2.,10.,5.,12.]  [1.,10.,5.,12.] [10.,10.,5.,12.])

nt=2; # number of combinations of planners and knowns

NUMTESTS=10;

rl=1; # cannot be a float
ru=10; # cannot be a float
#nr=10;
declare -a radi
for ((i=0;i<NUMTESTS;i+=1));
do
  radi[${i}]=[$((rl+RANDOM%(ru-rl))).$((RANDOM%99)),10.,5.,12.]
done
echo "the radius vectors are: "${radi[*]}

vl=0; # cannot be a float
vu=20; # cannot be a float
#nv=10;
declare -a vys
for ((i=0;i<NUMTESTS;i+=1));
do
  vys[${i}]=[-$((vl+RANDOM%(vu-vl))).$((RANDOM%99)),0.,0.,-1.] # notice the negative sign!
done
echo "the velocity vectors are: "${vys[*]}

INITIAL_TIME=$SECONDS

convertsecs() {
 ((h=${1}/3600))
 ((m=(${1}%3600)/60))
 ((s=${1}%60))
 printf "%02d:%02d:%02d\n" $h $m $s
}

for ((idx=0;idx<NUMTESTS;idx+=1));
do
  ${vy}
  num=$(( NUMTESTS*nt ))

  echo ${vys[$idx]}
  echo ${radi[$idx]}

  echo "Running for the  $(( ${idx} + 1 )) th time out of $num"

  TIME1=$(( ($SECONDS-INITIAL_TIME) / (idx+1) * (num-(idx+1)) ))

  echo "--------------------------------------------------------------"
  echo "Estimated time (hours, minutes, seconds) remaining is: "
  echo $(convertsecs $TIME1)
  echo "_____________________________________________________________"

done
