CXX = g++
CXXFLAGS = -Wall -std=c++17 \
	-I../src/core \
	-I../src/physics \
	-I../src/collision \
	-I../src/geometry \
	-I/opt/homebrew/opt/sfml/include

LDFLAGS = -L/opt/homebrew/opt/sfml/lib -lsfml-graphics -lsfml-window -lsfml-system

SRC = main.cpp \
	../src/physics/RigidBody.cpp \
	../src/collision/Collision.cpp

TARGET = physics_demo

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
