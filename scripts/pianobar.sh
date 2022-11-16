#!/bin/ksh

shutdown() {
  if [ "$BPID" != "" ];then
    kill -9 $BPID
  fi
  exit 0
}

trap shutdown 0 1 2 3 9 15

if [ -d ~/defaults/pianobar ];then
  echo copying defaults
  cp -rp ~/defaults/pianobar ~/.config
fi

pianobar $* &
BPID=$!

wait
