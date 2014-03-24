
INCLUDE=-I ./v1.0/ardupilotmega -I ./bcm2835-1.36/src

all: camera

camera: main.c ./bcm2835-1.36/src/bcm2835.o
	g++ $(INCLUDE) -Wall -o $@ $^

clean:
	rm -f camera
