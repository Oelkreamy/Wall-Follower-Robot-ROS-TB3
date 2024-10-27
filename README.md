# ROS TurtleBot3 Wall-Following Project

**ROS TurtleBot3 Wall-Following Project** is a robotics application developed using ROS (Robot Operating System), This project demonstrates a comprehensive application of ROS concepts, designed to control a TurtleBot3 robot in both simulated and real-world environments. The primary goal of this project is to showcase foundational ROS skills, including topic publishing/subscribing, service creation, and action server implementation. Each section builds upon the previous to create a versatile and fully functional robot capable of autonomous wall-following and odometry recording.

## ✨ Project Highlights

- **Intelligent Wall-Following**: Leverages real-time laser scan data to allow the TurtleBot3 to autonomously follow a wall while dynamically adjusting its distance based on predefined safe zones.
- **Dynamic Wall Detection Service**: Created a custom ROS service to identify and approach the nearest wall, seamlessly integrating with the wall-following algorithm for enhanced situational awareness.
- **Odometry Tracking Action Server**: Implements a ROS action server to track and record the robot’s odometry, providing real-time feedback on traveled distances with high precision.
  
## ⚙️ Technologies and Frameworks

- **ROS (Robot Operating System)**: Core middleware enabling modular robotics control and real-time communication.
- **Gazebo & Rviz**: Used for simulation, enabling seamless testing in a controlled environment before deploying on physical hardware.
- **Python**: Primary language for developing ROS nodes, services, and action servers, allowing clean and efficient robotic control logic.
- **Actionlib and Servicelib**: ROS libraries for action and service servers, essential for asynchronous and modular task management.

## 📂 Project Structure

The project is organized to maximize modularity and reusability, with each component separated based on functionality:

### Core Files

- **`default.py`**: Provides detailed instructions for project setup, launching, and testing. Essential for understanding the flow and requirements of the project.
  
- **`FindWall.srv`**: Defines a custom ROS service for wall detection, making the robot orient itself toward and approach the nearest wall.
  
- **`OdomRecord.action`**: Contains the ROS action definition for odometry tracking, allowing real-time feedback on distances traveled and providing accurate feedback for distance-based tasks.

### Key Nodes

- **`action_server.py`**: Implements the odometry recording action server. Records the robot’s positional data in (x, y, theta) format and calculates the cumulative distance, allowing dynamic tracking during wall-following.

- **`service_server.py`**: Contains the logic for the wall-detection service server. Uses laser scan data to determine the nearest wall and position the robot for optimal wall-following, initializing a safe starting point.

### Launch Files

- **`project_launch_file.launch`**: Main launch file to initialize and coordinate all nodes, services, and actions, setting up the complete wall-following system.
- **`another.launch`**: Additional configurations for flexible testing and development environments.

### Configuration Files

- **`CMakeLists.txt`** and **`package.xml`**: Define the project’s build configuration and dependencies, ensuring seamless integration with the ROS ecosystem.

## 🌐 System Flow

1. **Simulation Launch**: Begins with initializing the Gazebo and Rviz environment.
2. **Service and Action Initialization**: Service and action servers (`FindWall` and `OdomRecord`) are launched, ready to respond to calls from the wall-following algorithm.
3. **Wall-Detection and Orientation**: Using laser scan data, the robot identifies the nearest wall, aligns its orientation, and moves forward until it reaches the designated starting distance.
4. **Adaptive Wall-Following**: The robot maintains a specified distance from the wall while adjusting its path to avoid obstacles or changes in wall direction, transitioning between walls as needed.
5. **Odometry Feedback**: The odometry action server tracks and logs the robot’s path, providing precise data on distance traveled, which is essential for tasks requiring path validation or distance-based analysis.

## 🧠 Skills Demonstrated

- **ROS Integration**: Demonstrates a complete ROS project with custom services and actions, showcasing a range of ROS capabilities from simple publishing to advanced action servers.
- **Advanced Robotics Logic**: Implements intricate wall-following behavior, requiring an understanding of sensor data, feedback mechanisms, and environmental mapping.
- **Code Modularity and Extensibility**: Designed with clean separation of components, facilitating future expansions, testing, and debugging.
- 
## 📸 Real-World Application with The Construct’s Remote Real Robots

This project was successfully deployed on a real TurtleBot3 robot using **The Construct’s Remote Real Robots** service, which provides access to real robots in a remote lab environment. 

### The Construct’s ROSject Platform
The code and simulation for this project were developed on **The Construct’s ROSject** platform, which offers a comprehensive cloud-based ROS development environment, allowing for easy transitions from simulation to real-world deployment.

### Project in Action

Below are photos from the real-world deployment of the project, showcasing the TurtleBot3 performing wall-following in The Construct’s remote lab:

| ![Moving to Start Position](https://drive.google.com/uc?id=1mWr5p9l7TewPDp6zsZDXIkLwjRndhF1k) | ![Wall Detection Initialization](https://drive.google.com/uc?id=1rLFAZhISQOLOi42H2vMGUnamLjgy10ml) | ![Odometry Feedback Display](https://drive.google.com/uc?id=1jgk8ovoN-6d8Xs6q9mAYZfbc3O6ClS1c) |
|:--:|:--:|:--:|
| *Moving the robot to Start Position* | *Robot initializing wall detection* | *Live odometry tracking feedback* |

## 🚀 Installation & Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/turtlebot3-wall-following.git
   cd turtlebot3-wall-following
   ```

2. **Build the Workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. **Source the Workspace**:
   ```bash
   source devel/setup.bash
   ```

4. **Launch the System**:
   ```bash
   roslaunch yourpackage project_launch_file.launch
   ```

## 🤝 Contributions

Contributions to this project are welcome. Please open a pull request or submit an issue for any suggestions, improvements, or bug reports.

## 📜 License

Licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
