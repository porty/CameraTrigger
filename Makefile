
INCLUDE=-I ./v1.0/ardupilotmega -I ./bcm2835-1.36/src

all: camera

camera: main.c
	g++ $(INCLUDE) -l bcm2835 -Wall -o $@ $^

clean:
	rm -f camera
