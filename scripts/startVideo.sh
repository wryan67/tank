#!/bin/ksh

BINDIP=192.168.33.27

#while [ 1 ];do
  #raspivid -t 0 -w 1920 -h 720 -fps 4 -l -o tcp://0.0.0.0:8080

#  raspivid -t 0 -b 120000 -w 640 -h 480 -fps 24 -o - | nc -p 1980 -u 192.168.33.27 8080 
#  raspivid -t 0 -b 2000000 -fps 30 -w 800 -h 600 -o - | nc -p 1904 -u 192.168.2.108 1234

#  raspivid -t 0 -n -b 1000000 -w 640 -h 480 -fps 24 -o - | \
#      cvlc -v --open stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8080/}' :demux=h264


# raspivid -o - -t 0 -w 800 -h 600 -fps 12  | cvlc --sout '#rtp{sdp=rtsp://0.0.0.0:8080/}' :demux=h264


#raspivid -fps 24 -h 450 -w 600 -vf -n -t 0 -b 200000 -o - |\
#  gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 \
#                    gdppay ! tcpserversink host=$BINDIP port=8080


# raspivid -t 0 -w 640 -h 480 -fps 24 -b 1200000 -p 0,0,640,480 -o - | \
#	 gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 \
#	                         ! gdppay ! tcpserversink host=0.0.0.0 port=8080
 
#done
#gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=-1 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=192.168.33.27 port=5200

WIDTH=1280
HEIGHT=720

WIDTH=800
HEIGHT=480

IP=192.168.33.27 
IP=192.168.22.252

PORT=5000

while [ 1 ];do
  gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=-1 \
	  ! video/x-raw,width=${WIDTH},height=${HEIGHT}, framerate=24/1 \
	  ! videoconvert \
	  ! jpegenc \
	  ! tcpserversink host=$IP port=$PORT
done
