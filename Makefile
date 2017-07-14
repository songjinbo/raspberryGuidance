CXX = g++
TARGET = getData
OBJECTS = getDataAndTrans.o DJI_utility.o
CFLAGS = -g -Wall -I../include `pkg-config --cflags opencv`
LDFLAGS = -Wl,-rpath,./:/usr/local/lib -lpthread -lrt  -L. -L/usr/local/lib/ -lDJI_guidance -lusb-1.0 `pkg-config --libs opencv`

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
getDataAndTrans.o : getDataAndTrans.cpp  DJI_utility.h
	$(CXX) $(CFLAGS) -c getDataAndTrans.cpp
DJI_utility.o :
	$(CXX) $(CFLAGS) -c DJI_utility.cpp
clean:
	rm -rf *.o *.gch *.avi $(TARGET)
