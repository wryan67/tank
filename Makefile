
all:  knobtest 

knobtest: main.cpp
	gcc -O2 -lwiringPi -lwiringPiADS1115rpi -lwiringPiMCP23x17rpi main.cpp -o main

clean:
	rm -f main 

