
https://github.com/user-attachments/assets/6b15c781-90f8-40a2-9a30-b9eb64c5afaa

https://github.com/user-attachments/assets/6116e626-a024-46ee-8ee4-7a5415dcd209



# Robot Control System
## Overview

This project involves developing a comprehensive robot control system using C++ and OpenGL. The system integrates vision and lidar technologies to scan objects, visualize the results, and perform inspection tasks. It encompasses real-time object detection, lidar-based scanning, and interactive 3D visualization.

The project includes the following major functionalities:
- **Vision System**: Utilizes computer vision techniques for object detection and recognition.
- **Lidar Integration**: Employs lidar sensors to scan the environment and generate 3D point clouds.
- **3D Visualization**: Implements OpenGL for real-time rendering of the robot's environment and scanned objects.
- **Robot Control**: Provides an interface for controlling the robot's movements and operations.
- **Inspection**: Includes tools for inspecting and analyzing scanned objects.

## Features

- **Vision System**: Uses OpenCV for real-time object detection and classification, and includes tools for camera calibration to ensure accurate measurements.
- **Lidar Integration**: Continuously scans the environment to create 3D maps and processes point cloud data for visualization and analysis.
- **3D Visualization**: Renders the robot's environment using OpenGL, with an interactive interface that allows users to zoom, rotate, and interact with the 3D visualization.
- **Robot Control**: Offers manual control via keyboard inputs and an autonomous mode for automatic scanning and inspection.
- **Inspection**: Tools for detailed inspection and analysis of scanned objects.

## Getting Started

### Prerequisites

- **C++ Compiler**: Ensure you have a C++ compiler installed (e.g., GCC, Clang, MSVC).
- **OpenGL**: Install OpenGL and related libraries (GLFW, GLAD).
- **OpenCV**: Required for the vision system.
- **Lidar SDK**: Install the SDK provided by your lidar sensor manufacturer.
- **YMconnect SDK**: Install the SDK for robot control.

### Usage

#### Robot Control

- **Manual Control**: Use keyboard inputs to control the robot's movements.
- **Autonomous Mode**: Enable autonomous mode for automatic scanning and inspection.

#### Vision System

- **Object Detection**: Utilizes OpenCV to detect and classify objects in real time.
- **Camera Calibration**: Tools for calibrating the camera to ensure accurate measurements.

#### Lidar Scanning

- **Real-time Scanning**: Continuously scans the environment to create a 3D map.
- **Point Cloud Processing**: Processes the point cloud data for visualization and analysis.

#### 3D Visualization

- **Environment Rendering**: Renders the robot's environment using OpenGL.
- **Interactive Interface**: Allows users to interact with the 3D visualization (e.g., zoom, rotate).

## My Contributions

As the primary C++ developer for this project, my contributions include:

- **Development of Core Software**: Implemented the main application logic and integrated various components using C++.
- **OpenGL Integration**: Developed the 3D visualization features, including real-time rendering of the robot's environment and scanned objects.
- **ImGui Integration**: Implemented the user interface using ImGui to provide a user-friendly experience.
- **YMconnect SDK Integration**: Integrated the YMconnect SDK to control the robot's movements and operations.
- **IPC Implementation**: Developed the Inter-Process Communication (IPC) mechanisms to connect with Python scripts from other team members, ensuring seamless data transfer and coordination between different modules.
- **System Architecture**: Designed the overall system architecture to ensure modularity, scalability, and maintainability.

## Contributing

We welcome contributions from the community! If you would like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes and push the branch to your fork.
4. Open a pull request to the main repository.

## License

This project is licensed under the MIT License.

## Acknowledgements

- **OpenGL**: For providing the graphics rendering capabilities.
- **ImGui**: For the user interface framework.
- **PCL**: For point cloud analysis.
- **IPC**: For Inter-Process Communication for data transferring inside the system.
- **OpenCV**: For the vision system and image processing.
- **Lidar SDK**: For enabling lidar sensor integration.
- **YMconnect SDK**: For robot control capabilities.

## Contact

For any questions or inquiries, please contact:
email: [ngdtuan.dn@gmail.com].
LinkedIn: https://www.linkedin.com/in/nguyen-tuan-a2a589128/
We hope you find this project useful and look forward to your contributions and feedback!


