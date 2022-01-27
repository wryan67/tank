#!/bin/bash

#ps -fu `whoami` | awk '{if ((/arecord/ || /aplay/) && !/awk/) system(sprintf("kill -9 %d",$2))}'

BUFFER_SIZE=64 

export AUDIODEV="dmix:CARD=Ultimate,DEV=0"
export AUDIODEV="dmix:CARD=BT700,DEV=0"
export AUDIODEV="dmix:CARD=Device,DEV=0"

MIC=`arecord -l | sed -e 's/://' -ne "/USB audio CODEC/p" | awk '{print $2}'`

[ "$MIC" = "" ] && MIC=`arecord -l | sed -e 's/://' -ne "/Device .USB Audio Device/p" | awk '{print $2}'`
[ "$MIC" = "" ] && MIC=`arecord -l | sed -e 's/://' -ne "/Ultimate/p" | awk '{print $2}'`

INPUT="hw:${MIC},0"

SPEAKER=`aplay -l | sed -e 's/://' -ne "/Poly BT700/p" | awk '{print $2}'`

OUTPUT="hw:${SPEAKER},0"


AUDIODEV=""
AUDIODEV=`aplay -L | grep ^dmix | grep BT700`
[ "$AUDIODEV" = "" ] && AUDIODEV=`aplay -L | grep ^dmix | grep Ultimate`
[ "$AUDIODEV" = "" ] && AUDIODEV=`aplay -L | grep ^dmix | grep Device`

#OUTPUT=$AUDIODEV

RATE=48000

echo MIC=$INPUT
echo OUT=$OUTPUT

while [ 1 ];do
(
arecord -D $INPUT -c1 -f S16_LE --rate=$RATE --file-type=raw  --buffer-size=$BUFFER_SIZE | 
    sox -q --buffer $BUFFER_SIZE \
        -t raw -e signed-integer -L -r $RATE -c1 -b16 - \
        -t raw -e signed-integer -L -r 48000 -c2 -b16 - |
  aplay -D $OUTPUT -c2 -f S16_LE --rate=48000 --file-type=raw --buffer-size=$BUFFER_SIZE -

)2>&1 | egrep -v "under-run|underrun"
done

