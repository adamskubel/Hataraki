CXX=g++
CXXFLAGS=-g -std=c++0x 
	
LIB=rt
LIB_PARAMS=$(foreach d,$(LIB),-l$d)

LIBDIR=
LIB_DIR_PARAMS=$(foreach l, $(LIBDIR),-L$l)

INC=
INC_PARAMS=$(foreach d, $(INC),-I$d)

MAINFILE=Main.cpp
TESTFILE=Test.cpp

SOURCES=DRV8830.cpp AS5048.cpp I2CBus.cpp PredictiveJointController.cpp cJSON.cpp ikfastsolution.cpp Configuration.cpp MotionController.cpp TimeMultiplexedVoltageConverter.cpp vmath.cpp MathUtils.cpp ServoUtil.cpp PoseDynamics.cpp TimeUtil.cpp AsyncLogger.cpp

CLEAN=Main.cpp DRV8830.cpp AS5048.cpp I2CBus.cpp PredictiveJointController.cpp Configuration.cpp MotionController.cpp TimeMultiplexedVoltageConverter.cpp MathUtils.cpp ServoUtil.cpp PoseDynamics.cpp TimeUtil.cpp AsyncLogger.cpp

OBJECTS=$(SOURCES:.cpp=.o)
CLEANOBJ=$(CLEAN:.cpp=.o)
MAINOBJ=$(MAINFILE:.cpp=.o)
TESTOBJ=$(TESTFILE:.cpp=.o)

EXECUTABLE=BasicMotion
TEST_EXECUTABLE=TestMotion

default: all

all:  $(MAINFILE) $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(MAINOBJ) $(OBJECTS)
	$(CXX) -g $(LIB_DIR_PARAMS) $(LIB_PARAMS) -o $@ $(MAINOBJ) $(OBJECTS)

test: $(TESTFILE) $(SOURCES) $(TEST_EXECUTABLE)

$(TEST_EXECUTABLE): $(TESTOBJ) $(OBJECTS)
	$(CXX) $(LIB_DIR_PARAMS) $(LIB_PARAMS) -o $@ $(TESTOBJ) $(OBJECTS)

clean:
	rm -f $(CLEANOBJ) $(MAINOBJ) $(TESTOBJ) $(EXECUTABLE) $(TEST_EXECUTABLE)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INC_PARAMS) -c -o $@ $<
