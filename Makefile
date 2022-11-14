
# debugging
#DEBUG  = -g -O0 -rdynamic -std=c++17
# production
DEBUG = -O2 

CFLAGS=-std=c++17 

all:  tank 

SYSLIBS=-lm -luuid -pthread
DEVICELISBS=-lNeoPixelRPi -lwiringPi -lwiringPiADS1115rpi -lwiringPiPCA9635rpi -lwiringPiPca9685 -lwiringPiMCP23x17rpi
GSTLIB=`pkg-config --cflags --libs gstreamer-1.0`
LIBS=${SYSLIBS} ${DEVICELISBS} ${GSTLIB} -llog4pi 


tank: main.cpp 
	g++ ${CFLAGS} ${DEBUG} ${LIBS} main.cpp -o tank

# mcp23x17_threads.o: mcp23x17_threads.c mcp23x17_threads.h
# 	gcc ${DEBUG} -c mcp23x17_threads.c 

# mcp23x17rpi.o: mcp23x17rpi.c mcp23x17rpi.h
# 	gcc ${DEBUG} -c mcp23x17rpi.c 


clean:
	rm -f tank 

