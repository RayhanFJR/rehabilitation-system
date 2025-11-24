# Makefile untuk Sistem Kontrol Rehabilitasi

CXX = g++
CXXFLAGS = -std=c++11 -Wall -O2
LDFLAGS = -lboost_system -lmodbus -lpthread

# Source files
SOURCES = main.cpp \
          TrajectoryData.cpp \
          SerialCommunication.cpp \
          ModbusHandler.cpp \
          SystemController.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
TARGET = rehab_control

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Clean complete"

# Rebuild from scratch
rebuild: clean all

# Install (optional)
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/
	@echo "Installed to /usr/local/bin/$(TARGET)"

# Run the program
run: $(TARGET)
	./$(TARGET)

# Help
help:
	@echo "Available targets:"
	@echo "  all      - Build the program (default)"
	@echo "  clean    - Remove build files"
	@echo "  rebuild  - Clean and build"
	@echo "  install  - Install to /usr/local/bin"
	@echo "  run      - Build and run the program"
	@echo "  help     - Show this help message"

.PHONY: all clean rebuild install run help