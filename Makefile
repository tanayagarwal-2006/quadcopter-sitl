CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -Icontroller
LIBS = -lws2_32

TARGET = sitl_controller

SRC = \
	controller/angle_controller.cpp \
	controller/pid_rate_controller.cpp \
	controller/motor_mixer.cpp \
	controller/state_estimator.cpp \
	controller/udp_bridge.cpp \
	controller/utils.cpp \
	testbenches/full_pipeline_testbench.cpp

OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET) $(TARGET).exe