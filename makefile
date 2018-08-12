all: main.cpp LoRa.o
	g++ -o main main.cpp LoRa.o -lwiringPi -Wall

LoRa.o: LoRa.h LoRa.cpp
	g++ -c LoRa.cpp -lwiringPi -Wall

clean:
	rm *.o main