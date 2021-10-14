
# debugging
#DEBUG  = -g -O0 -rdynamic
# production
DEBUG = -O2

all:  tank 

LIBS=-lwiringPi -lwiringPiADS1115rpi -lwiringPiPCA9635rpi -lwiringPiMCP23x17rpi


tank: main.cpp 
	gcc ${DEBUG} ${LIBS} main.cpp -o tank

# mcp23x17_threads.o: mcp23x17_threads.c mcp23x17_threads.h
# 	gcc ${DEBUG} -c mcp23x17_threads.c 

# mcp23x17rpi.o: mcp23x17rpi.c mcp23x17rpi.h
# 	gcc ${DEBUG} -c mcp23x17rpi.c 


clean:
	rm -f tank 

