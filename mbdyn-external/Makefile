CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

SOURCE =    mbdyn_external.cpp

OBJS =		mbdyn_external.o


INCLUDES = -I/home/claudio/Projects/PreCICE/mbdyn/include

LIBS = -L/home/claudio/Projects/PreCICE/mbdyn/lib -lmbc

TARGET =	mbdyn_external


$(OBJS):	$(SOURCE)
	$(CXX) -c -o $@ $< $(INCLUDES)

$(TARGET):	$(OBJS)
	$(CXX) -o bin/$(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) bin/$(TARGET)
