CC=gcc
CXX=g++
CXXFLAGS+=-O2
INCLUDE +=-I ./include/openni2
LIBS +=-lOpenNI2 -lOpenNI2.jni -lwiringPi -lpthread
follow:
	$(CXX) -o follow follow_pwm.cpp $(INCLUDE) $(LIBS) $(CXXFLAGS)
clean:
	rm -rf follow follow.o

