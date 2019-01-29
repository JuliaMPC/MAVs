#!/bin/bash
# $0 is the script name, $1 id the first ARG, $2 is second...

if [ "$2" = "" ]
then
  echo "Usage: $0 <filename> <namespace>"
else
  dump_command="rosparam dump"
  load_command="rosparam load"

  FILE="$1"
  NAMESPACE="$2"
  TMPFILE="$3"

  IS_NAMESPACE=$(rosparam list | /bin/egrep "$NAMESPACE")

  if [ -z "$IS_NAMESPACE" ];
  then
    echo "No previous Namespace found, simply loading new parameters"
    command="$load_command $FILE $NAMESPACE"
    eval $command
  else

    #Dumps the current parameters in the server
    command="$dump_command dump_tmp.txt $NAMESPACE"
    eval $command

    #Temporarly loads the new parameters
    command="$load_command $FILE $NAMESPACE"
    eval $command

    #Dumps the new parameters
    command="$dump_command load_tmp.txt $NAMESPACE"
    eval $command

    #Appends both files
    cat dump_tmp.txt >> load_tmp.txt

  #  echo `cat load_tmp.txt`

    #Loads all the new parameters
    command="$load_command load_tmp.txt $NAMESPACE"
    eval $command

    #Dumps all the new parameters
    command="$dump_command $TMPFILE $NAMESPACE"
    eval $command

    #Remove temporary files
    rm load_tmp.txt
    rm dump_tmp.txt
  fi
fi
