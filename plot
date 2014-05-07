#!/bin/bash

if [ "$1" == "-l" ]
then
    plotName=$2
else
    plotName=$1
    rsync dione:repos/research/*/$plotName.m .
fi

cat $plotName.m | ssh clear -Y "bash -l -c \"sleep 1; math\"" > /dev/null 2> /dev/null

rsync clear:$plotName.pdf . 2> /dev/null
ssh clear "rm $plotName.pdf" 2> /dev/null
