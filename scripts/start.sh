#!/bin/ksh
set -a

~/bin/stop.sh
~/bin/stopAudio.sh

FAST=0

if [ "$1" = "-f" ];then
  FAST=1
  shift
else
  FAST=0
fi

[ $FAST = 0 ] && sudo $HOME/bin/rainbow -b 128 &
[ $FAST = 0 ] && sleep 20

ps -ef | awk '{if (/rainbow/ && !/awk/ && !/sudo/) system(sprintf("sudo kill %d",$2))}'
[ $FAST = 0 ] && sleep 1.5

nohup sudo $HOME/bin/tank $* &

sleep 10
nohup ~/bin/recordTank.sh > /dev/null 2>&1 &

sleep 30
nohup ~/bin/recordHeadset.sh > /dev/null 2>&1 &
