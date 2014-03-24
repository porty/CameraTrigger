
INCLUDE=-I ./v1.0/ardupilotmega -I ./bcm2835-1.36/src

all: camera

camera: main.c ./bcm2835-1.36/src/bcm2835.o
	g++ $(INCLUDE) -Wall -o $@ $^

bcm2835-1.36/src/bcm2835.o: bcm2835-1.36/src/Makefile
	cd bcm2835-1.36/src && make

bcm2835-1.36/src/Makefile: bcm2835-1.36/config.h

bcm2835-1.36/config.h:
	cd bcm2835-1.36 && ./configure -q

clean:
	rm -f camera bcm2835-1.36/config.h bcm2835-1.36/src/Makefile bcm2835-1.36/src/bcm2835.o
