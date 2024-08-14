CXX = g++
CXXFLAGS = -Iinclude -std=c++11 -Wall -Wextra ${DEBUG_FLAGS}

SRC_DIR = src
BUILD_DIR = build

MAIN_SRC = $(SRC_DIR)/imu_driver.cpp
IMU_SRC = $(SRC_DIR)/dummy_imu.cpp

MAIN_OBJ = $(BUILD_DIR)/imu_driver.o
IMU_OBJ = $(BUILD_DIR)/dummy_imu.o

IMU_DRIVER_APP = $(BUILD_DIR)/imu_driver
IMU_APP = $(BUILD_DIR)/imu_sim

# Targets
all: $(IMU_DRIVER_APP) $(IMU_APP)
debug: $(IMU_DRIVER_APP) $(IMU_APP)

# Add debug options
debug: DEBUG_FLAGS = -DDEBUG -g -O0

# build imu_driver executable
$(IMU_DRIVER_APP): $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $(MAIN_OBJ)

# build imu_sim executable
$(IMU_APP): $(IMU_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $(IMU_OBJ)

# Compile imu_driver.cpp to object file
$(BUILD_DIR)/imu_driver.o: $(SRC_DIR)/imu_driver.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Compile dummy_imu.cpp to object file
$(BUILD_DIR)/dummy_imu.o: $(SRC_DIR)/dummy_imu.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Clean build files
clean:
	rm -f $(BUILD_DIR)/*.o $(IMU_DRIVER_APP) $(IMU_APP)