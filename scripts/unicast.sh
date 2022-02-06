#!/bin/ksh
set -a

WIDTH=$1 
HEIGHT=$2

FPS=24
PORT=5000

PATH=$PATH:/usr/sbin

if [ $# -lt 2 ];then
  echo "usage: `basename $0` [width] [height]"
  exit 2
fi 

shutdown() {
  if [ "$BPID" != "" ];then
    kill -9 $BPID
  fi
  exit 0
}

trap shutdown 0 1 2 3 9 15


launch() {
  [ "$CLIENTS" = "" ] && return
 
  if [ "$BPID" != "" ];then
    kill -9 $BPID
    sleep 0.5
  fi

  echo launching $CLIENTS
  LAST="$CLIENTS"
  mkdir -p $HOME/logs

  nohup gst-launch-1.0 -v v4l2src device=/dev/video0 \
       ! video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 \
       ! clockoverlay time-format="%D %H:%M:%S" \
       ! jpegenc \
       ! rtpjpegpay \
       ! udpsink clients="${CLIENTS}" > $HOME/logs/gst.log 2>&1 &
  BPID=$!

}

getClients() {
  FIRST=1
  CLIENTS=""
  for i in $(arp -eni wlan1 | sed -ne 's/^\([0-9\.]*\).*/\1/p')
  do
    if [ $FIRST = 1 ];then
      FIRST=0 
    else
      CLIENTS="${CLIENTS},"
    fi    
    CLIENTS="${CLIENTS}${i}:$PORT"
  done

}

getClients
launch

while [ 1 ];do
   if [ "$LAST" != "$CLIENTS" ];then
     launch
   fi
        
   sleep 10
   getClients
done
