#!/bin/ksh

#ps -fu `whoami` | awk '{if ((/arecord/ || /aplay/) && !/awk/) system(sprintf("kill -9 %d",$2))}'

BUFFER_SIZE=64 


SPEAKER=`aplay -l | sed -e 's/://' -ne "/.*Device .USB Audio Device/p" | awk '{print $2}'`

MIC=`aplay -l | sed -e 's/://' -ne "/Poly BT700/p" | awk '{print $2}'`

[ "$MIC" = "" ] && MIC=`aplay -l | sed -e 's/://' -ne "/Ultimate/p" | awk '{print $2}'`


INPUT="hw:${MIC},0"

RATE=48000
RATE=16000

echo SPEAKER=$SPEAKER

#RECORD_OPTS="--period-size=32 -F 2000 -B 4000 "
#RECORD_OPTS="-B 4000 -F 2000"

PIPE=/tmp/headsetRecorder.wav
rm -rf $PIPE
mknod  $PIPE p

export AUDIODEV="dmix:CARD=Ultimate,DEV=0"
export AUDIODEV="dmix:CARD=BT700,DEV=0"
export AUDIODEV="dmix:CARD=Device,DEV=0"

OUTPUT="dmix:CARD=Device,DEV=0"
OUTPUT="hw:${SPEAKER},0"

echo MIC=$INPUT
echo OUT=$OUTPUT

while [ 1 ];do
(
  arecord -D $INPUT  -c1 -f S16_LE --rate=$RATE --file-type=raw --buffer-size=$BUFFER_SIZE | 
    sox -q --buffer $BUFFER_SIZE \
        -t raw -e signed-integer -L -r $RATE -c1 -b16 - \
        -t raw -e signed-integer -L -r 44100 -c2 -b16 - |
    aplay -D $OUTPUT -c2 -f S16_LE --rate=44100 --file-type=raw --buffer-size=$BUFFER_SIZE - 
)2>&1 | egrep -v "under-run|underrun|overrun"

done



