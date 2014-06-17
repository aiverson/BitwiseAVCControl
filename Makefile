all: nibble

nibble: Comm.o Mission.o ComputerVision.o nibble.o 
	g++ -pthread nibble.o Comm.o Mission.o ComputerVision.o -L/usr/local/lib -L/usr/local/cuda-6.0/lib `pkg-config --libs opencv` -o nibble

nibble.o: nibble.cpp
	g++ -std=gnu++11 -I mavlink/include -c nibble.cpp

Comm.o: Comm.cpp
	g++ -I mavlink/include -c Comm.cpp

Mission.o: Mission.cpp
	g++ -I mavlink/include -c Mission.cpp

ComputerVision.o: ComputerVision.cpp
	g++ -std=gnu++11 -L/usr/local/lib -L/usr/local/cuda-6.0/lib `pkg-config --cflags opencv` -c ComputerVision.cpp 
 
clean:
	rm -rf *o nibble
