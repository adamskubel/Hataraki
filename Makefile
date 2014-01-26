CXX=g++
CXXFLAGS=-w -g -std=c++0x 
	
LIB=rt
LIB_PARAMS=$(foreach d,$(LIB),-l$d)

LIBDIR=
LIB_DIR_PARAMS=$(foreach l, $(LIBDIR),-L$l)

INC=
INC_PARAMS=$(foreach d, $(INC),-I$d)

#MAINFILE=Main.cpp
#TESTFILE=Test.cpp
SOURCES=Main.cpp DRV8830.cpp AS5048.cpp I2CBus.cpp JointLoop.cpp cJSON.cpp ikfastsolution.cpp Configuration.cpp MotionController.cpp SpeedController.cpp
OBJECTS=$(SOURCES:.cpp=.o)

EXECUTABLE=BasicMotion

default: all

all:  $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LIB_DIR_PARAMS) $(LIB_PARAMS) -o $@ $(OBJECTS)

#test: $(SOURCES) $(EXECUTABLE)

#$(EXECUTABLE): $(OBJECTS)
#	$(CXX) $(LIB_DIR_PARAMS) $(LIB_PARAMS) -o $@ $(OBJECTS)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INC_PARAMS) -c -o $@ $<
