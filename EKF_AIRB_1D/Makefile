CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++14

SRC_CPP = app.cpp
OBJ = $(SRC_CPP:.cpp=.o)
EXECUTABLE = comp

all: $(EXECUTABLE) run_python

$(EXECUTABLE): $(OBJ)
	$(CXX) $(OBJ) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

output.csv: $(EXECUTABLE)
	./$(EXECUTABLE)

run_python: output.csv
	python data_vis.py

clean:
	rm -f $(OBJ) $(EXECUTABLE) KF.o IMU_sim.o output.csv
