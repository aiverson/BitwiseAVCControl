all: nibble

nibble: Comm.o Mission.o nibble.o
	g++ nibble.o Comm.o Mission.o -o nibble

nibble.o: nibble.cpp
	g++ -I mavlink/include -c nibble.cpp

Comm.o: Comm.cpp
	g++ -I mavlink/include -c Comm.cpp

Mission.o: Mission.cpp
	g++ -I mavlink/include -c Mission.cpp

clean:
	rm -rf *o nibble
