CXX_BB=g++
CXX=arm-none-linux-gnueabi-g++

OUTPUT_DIR=Build

CXXFLAGS=-g -O3 -std=c++0x --sysroot=/usr/local/carlson-minot/crosscompilers/arm-none-linux-gnueabi/libc-2013.05-24-sysroot

LIB=rt ncurses tinfo pthread
LIB_PARAMS=$(foreach d,$(LIB),-l$d)

LIBDIR=
LIB_DIR_PARAMS=$(foreach l, $(LIBDIR),-L$l)

INC=
INC_PARAMS=$(foreach d, $(INC),-I$d)

MAINFILE=Main.cpp
TESTFILE=VoltageTesting.cpp

SOURCES=DRV8830.cpp AS5048.cpp I2CBus.cpp PredictiveJointController.cpp Configuration.cpp MotionController.cpp TimeMultiplexedVoltageConverter.cpp MathUtils.cpp ServoUtil.cpp PoseDynamics.cpp TimeUtil.cpp AsyncLogger.cpp FlipIdentifier.cpp PredictiveJointControl_ControllerImpl.cpp PredictiveJointControl_SignalImpl.cpp vmath.cpp ikfastsolution.cpp cJSON.cpp MotionPlan.cpp MotionPlanner.cpp KinematicSolver.cpp QuadraticRegression.cpp IKControlUI.cpp UIElement.cpp NumberSpinner.cpp ServoModel.cpp TrajectoryPlanner.cpp ArmState.cpp TrajectoryController.cpp AsyncI2CSender.cpp ALog.cpp ServoDirectController.cpp WheelMotionController.cpp DriveModel.cpp RealtimeLoopController.cpp NavigationController.cpp ObstacleAwareNavigator.cpp AntennaDeflectionSensor.cpp

OBJECTS=$(SOURCES:.cpp=.o)
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
	rm -f $(OBJECTS) $(MAINOBJ) $(TESTOBJ) $(EXECUTABLE) $(TEST_EXECUTABLE)

#DEFAULT RULE OVERRIDE?? WTF!
.cpp.o:
	$(CXX) $(CXXFLAGS) $(INC_PARAMS) -c -o $@ $<
