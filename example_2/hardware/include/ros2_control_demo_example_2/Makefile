CC=g++
CFLAGS=-std=c++14 -Wall -Wextra -pedantic -g -O2 -pthread $$(pkg-config --cflags liblely-coapp)
LIBS=$$(pkg-config --libs liblely-coapp)

slave: slave.cpp
	$(CC) $(CFLAGS) -o slave slave.cpp $(LIBS)

master: master.cpp
	$(CC) $(CFLAGS) -o master master.cpp $(LIBS)

main: main.cpp
	$(CC) $(CFLAGS) -o main main.cpp $(LIBS)
