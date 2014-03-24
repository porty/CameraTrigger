all: camera

camera: main.c
	g++ -I ./v1.0/ardupilotmega -I ./bcm2835-1.36/src -o camera main.c -l bcm2835 -w

clean:
	rm -f camera
