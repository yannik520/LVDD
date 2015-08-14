all: v4l2videosink
CROSS_COMPILE ?=
CXX=$(CROSS_COMPILE)g++
CC=$(CROSS_COMPILE)gcc
LDFLAGS = -lavutil -lavformat -lavcodec -lswscale
LIB_SUFFIX=a
INCLUDES = -I./
CXXFLAGS= -c -g3 -O0 -fPIC $(INCLUDES)

v4l2videosink: v4l2videosink.o
	$(CXX) -o $@ $^ -fPIC -g3 -O0 -lpthread $(LDFLAGS)

cpp.o:
	$(CXX) $(CXXFLAGS) $<

clean:
	@rm -f *.o
	@rm -f v4l2videosink
