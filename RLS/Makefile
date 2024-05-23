CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++14 -g

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
	rm -f $(OBJ) $(EXECUTABLE) RLS.o output.csv
