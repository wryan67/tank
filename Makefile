
all:  knobtest 

knobtest: main.cpp
	gcc -O2 -lwiringPi -lwiringPiADS1115rpi -lwiringPiMCP23x17rpi -lwiringPiPCA9635rpi main.cpp -o tank

clean:
	rm -f tank 

