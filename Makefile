CC=gcc
CXX=g++
OBJS=follow.o serial.o
CXXFLAGS+=-g -O2
INCLUDE +=-I ./include/openni2
LIBS +=-lOpenNI2 -lOpenNI2.jni -lwiringPi -lpthread
follow:
	$(CXX) -o follow follow161202-4.cpp serial.cpp $(INCLUDE) $(LIBS) $(CXXFLAGS)
clean:
	rm -rf follow follow.o serial.o

