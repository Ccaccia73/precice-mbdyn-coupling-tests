CXXFLAGS =	-O0 -g -Wall -fmessage-length=0

SOURCE =    ts_driver.cpp

OBJS =		ts_driver.o


INCLUDES = #-I/home/claudio/Projects/PreCICE/mbdyn/include

LIBS = #-L/home/claudio/Projects/PreCICE/mbdyn/lib -lmbc

TARGET =	ts_driver


$(OBJS):	$(SOURCE)
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(INCLUDES)

$(TARGET):	$(OBJS)
	$(CXX) $(CXXFLAGS) -o bin/$(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) bin/$(TARGET)
