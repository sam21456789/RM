#!/bin/bash

ROOT_DIR="./result/${1}"

INDEX=`ls -l $ROOT_DIR | egrep '^d' | awk '{print $9}'`

function clear()
{
	rm -rf $1
	mkdir $1
}


# and now loop through the directories:
for index in $INDEX
do
	INDEX_DIR="$ROOT_DIR/$index"
	sub=`ls -l $INDEX_DIR | egrep '^d' | awk '{print $9}'`
	for d in $sub
	do
		des="$INDEX_DIR/$d"
		if [ -d $des ]
		then
			clear $des 
		else
			des="$INDEX_DIR/$d"
			if [ -d $des ]
			then
				clear $des
		else
			mkdir $des
		fi
	fi
	done
done
