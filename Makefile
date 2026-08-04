# Build and run the headless physics unit tests (no OpenGL / GLFW).
#
# The interactive simulator is built with CMake (it needs GLFW); this Makefile
# builds only the test suite, so CI can run it without a display.
#
#   make test    # compile and run the physics tests
#   make clean

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2 -Iinclude
HEADERS  := include/Vector2D.hpp include/Particle.hpp include/Simulation.hpp
TEST_BIN := physics_tests

.PHONY: test clean

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): tests/test_physics.cpp src/Simulation.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) tests/test_physics.cpp src/Simulation.cpp -o $(TEST_BIN)

clean:
	rm -f $(TEST_BIN)
