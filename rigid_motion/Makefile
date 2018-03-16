
# compiler flags
CXXFLAGS =	-O3 -g -Wall -fmessage-length=0 -std=c++11 #-fopenmp

#include paths
INCLUDE_DIRS = -I/home/claudio/Projects/PreCICE/precice/src/ -I/home/claudio/Projects/PreCICE/boost/1_66_0/include

# libraries
LIBS = -L/home/claudio/Projects/PreCICE/precice/build/last -L/home/claudio/Projects/PreCICE/boost/1_66_0/lib -lprecice -lboost_log -lboost_log_setup -lboost_thread -lboost_system -lboost_filesystem -lboost_program_options -lboost_regex

.PHONY: clean

all: RigidMotion


RigidMotion: RigidMotion.o
	g++ $(CXXFLAGS) -o bin/$@ $< $(LIBS)
	
RigidMotion.o: RigidMotion.cpp
	g++ $(CXXFLAGS) -c -o $@ $< -I/home/claudio/Projects/PreCICE/precice/src/

clean:
	rm -rf bin/RigidMotion RigidMotion.o
	
