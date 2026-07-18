CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -Icontroller
LIBS = -lws2_32

COMMON_SRC = \
	controller/angle_controller.cpp \
	controller/pid_rate_controller.cpp \
	controller/motor_mixer.cpp \
	controller/state_estimator.cpp \
	controller/udp_bridge.cpp \
	controller/utils.cpp

ROLL_SRC     = testbenches/roll_test.cpp
PITCH_SRC    = testbenches/pitch_test.cpp
CIRCULAR_SRC = testbenches/circular_flight_test.cpp

ROLL_TARGET     = roll_test
PITCH_TARGET    = pitch_test
CIRCULAR_TARGET = circular_flight_test

COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

all: $(ROLL_TARGET) $(PITCH_TARGET) $(CIRCULAR_TARGET)

$(ROLL_TARGET): $(COMMON_OBJ) $(ROLL_SRC:.cpp=.o)
	$(CXX) $^ -o $@ $(LIBS)

$(PITCH_TARGET): $(COMMON_OBJ) $(PITCH_SRC:.cpp=.o)
	$(CXX) $^ -o $@ $(LIBS)

$(CIRCULAR_TARGET): $(COMMON_OBJ) $(CIRCULAR_SRC:.cpp=.o)
	$(CXX) $^ -o $@ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(COMMON_OBJ) \
		testbenches/*.o \
		$(ROLL_TARGET) \
		$(PITCH_TARGET) \
		$(CIRCULAR_TARGET) \
		*.exe