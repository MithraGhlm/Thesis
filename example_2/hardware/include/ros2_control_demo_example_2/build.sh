g++ -std=c++14 -Wall -Wextra -pedantic -g -O2 canopen_comms.cpp -o canopen_coms $(pkg-config --libs liblely-io2) $(pkg-config --libs liblely-coapp) -lpthread
