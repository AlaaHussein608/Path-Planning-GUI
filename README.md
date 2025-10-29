# Path-Planning-GUI

A comprehensive graphical user interface for robot path planning and simulation. This application allows users to define environments with obstacles, set robot parameters, select from multiple path planning algorithms, and visualize both the planned path and robot motion through various plotting and animation tools.

## Features

### 🏗️ Environment Setup
- **Rectangle Obstacles**: Define rectangular obstacles with position and size parameters
- **Flexible Workspace**: Create custom environments by adding multiple obstacles

### 🤖 Robot Configuration
- **Start & Goal Positions**: Set initial and target positions for the robot
- **Visual Confirmation**: Points are visually confirmed in the environment

### 🧠 Path Planning Algorithms
Choose from five sophisticated path planning algorithms:

- **Dijkstra**: Classic shortest-path algorithm with guaranteed optimality
- **A***: Heuristic-based search with optimal path guarantee
- **Artificial Potential Fields (APF)**: Gradient-based approach using attractive and repulsive forces
- **Rapidly-exploring Random Tree (RRT)**: Sampling-based method for complex environments
- **RRT***: Optimal variant of RRT with asymptotic optimality

### 📊 Simulation & Visualization
- **Trajectory Plotting**: Visualize the complete planned path
- **Wheel Dynamics**: Monitor angular velocities of left and right wheels
- **Orientation Analysis**: Track robot orientation throughout the path
- **Real-time Animation**: Watch the robot navigate with orientation indicators

### 🎮 Control Strategies

**Cosine Switch Control** (Dijkstra & A*):
- High precision path following
- Stops at each waypoint for reorientation
- Ideal for structured environments with clear waypoints

**Proportional Control** (APF, RRT, RRT*):
- Continuous smooth motion
- Constant velocity operation
- Efficient for complex, curved paths
- Minimal stopping and reorientation

## Usage

### Set Up Environment:
- Enter obstacle parameters (position, width, height)
- Click "Add Rectangle" to place obstacles
- Repeat for multiple obstacles

### Configure Robot:
- Input start and goal coordinates
- Click "Set Points" to confirm

### Select Algorithm:
- Choose from the five available path planning methods
- Consider the environment complexity for algorithm selection

### Run Simulation:
- Click "Simulate" to generate plots:
  - Robot trajectory
  - Wheel angular velocities
  - Orientation over time
- Click "Animate" for real-time visualization:
  - Robot movement as a point
  - Red arrow indicating orientation
  - Continuous motion to goal

## Algorithm Recommendations

### For Simple Environments
- **Dijkstra/A***: Best for grid-based environments with optimal path requirements
- **Cosine Switch Control**: Provides pinpoint accuracy

### For Complex Environments
- **RRT/RRT***: Ideal for cluttered spaces and high-dimensional problems
- **APF**: Suitable for dynamic environments and real-time applications
- **Proportional Control**: Offers smooth, efficient motion
