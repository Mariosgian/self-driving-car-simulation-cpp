# Self-Driving Car Simulation

A C++ object-oriented simulation of a fully autonomous vehicle navigating a 2D GridWorld environment.

The system models core autonomous driving concepts including environment simulation, sensor perception, sensor fusion and GPS-based navigation.

---

# Overview

The program simulates a **Self-Driving Car** operating inside a grid-based world populated with both static and dynamic objects.

Objects in the environment include:

- Moving cars
- Moving bikes
- Parked cars
- Traffic lights
- Traffic signs

The autonomous vehicle:

- follows a sequence of **GPS targets**
- moves in **discrete time steps (ticks)**
- dynamically adjusts its speed depending on the distance to the target
- stops when the final target is reached or when simulation ticks are exhausted

The project is implemented using **object-oriented design principles** including:

- Encapsulation
- Inheritance
- Polymorphism
- Composition
- Communication between objects

---

# System Architecture

The simulation is built around several interacting components.

GridWorld
 ├── WorldObject
 │    ├── StaticObject
 │    │    ├── TrafficLight
 │    │    ├── TrafficSign
 │    │    └── StationaryVehicle
 │    │
 │    └── MovingObject
 │         ├── MovingCar
 │         └── MovingBike
 │
 └── SelfDrivingCar
      ├── NavigationSystem
      ├── SensorFusionEngine
      └── Sensors
           ├── LidarSensor
           ├── RadarSensor
           └── CameraSensor
Simulation Environment

The world is represented as a 2D grid managed by the GridWorld class.

Each object in the world derives from the abstract base class WorldObject, which provides:
	•	unique object IDs
	•	glyph representation for visualization
	•	position inside the grid
	•	virtual update() method

Objects are stored in:
std::vector<std::unique_ptr>

which allows polymorphic handling of all world objects.

---

# Autonomous Vehicle

The `SelfDrivingCar` class represents the autonomous vehicle and extends `MovingObject`.

It contains:

- a `NavigationSystem`
- a `SensorFusionEngine`
- multiple `Sensor` objects

The vehicle operates with three speed states:

STOPPED
HALF_SPEED
FULL_SPEED

Each simulation tick the vehicle:
	1.	Collects sensor readings
	2.	Processes them through the sensor fusion engine
	3.	Updates navigation decisions
	4.	Adjusts speed and direction
	5.	Moves toward the current GPS target

Navigation follows Manhattan distance logic.

⸻

Sensors

The vehicle is equipped with three sensor types.

Lidar Sensor
	•	Range: 9 cells
	•	Field of view: 360°
	•	Detects all objects
	•	High distance accuracy

Radar Sensor
	•	Range: 12 cells
	•	Detects only moving objects
	•	Provides speed and direction

Camera Sensor
	•	Range: 7 cells
	•	Detects all objects
	•	Reads traffic light color and traffic sign text

Each sensor produces SensorReading objects describing the detected environment.

⸻

Sensor Fusion

The SensorFusionEngine combines sensor readings from multiple sensors.

The fusion process:
	•	groups readings by object ID
	•	calculates weighted averages using sensor confidence
	•	merges qualitative attributes
	•	filters low-confidence readings

This produces a unified perception of the environment for the navigation system.

⸻

Visualization

Two visualization modes are implemented.

Full Visualization

Displays the entire GridWorld.

Vehicle POV Visualization

Displays a window around the vehicle showing nearby objects.

Visualization uses glyph priorities such as:

@  Self-Driving Car
R  Red Light
Y  Yellow Light
G  Green Light
S  Stop Sign
C  Moving Car
B  Moving Bike
P  Parked Car
.  Empty Cell
X  Outside Bounds

---

# Compilation

Compile using **g++ (C++17)**:

g++ -std=c++17 -O2 -Wall main.cpp -o simulator

Example Execution

./simulator –simulationTicks 30 
–numMovingCars 5 
–numMovingBikes 5 
–numTrafficLights 3 
–gps 10 10 30 5

The `--gps` parameter defines one or more navigation targets.

---

# Technologies

- C++
- Object-Oriented Programming
- Simulation Systems
- Sensor Fusion Concepts
