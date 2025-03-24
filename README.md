
https://github.com/user-attachments/assets/6b15c781-90f8-40a2-9a30-b9eb64c5afaa

https://github.com/user-attachments/assets/6116e626-a024-46ee-8ee4-7a5415dcd209


# Real-Time Robot Control and 3D Inspection System

## Project Overview

This project develops an integrated real-time robot control system leveraging modern C++, GPU acceleration, and visualization technologies (CUDA, OpenGL). The system incorporates computer vision and LiDAR sensors to facilitate precise object detection, environmental mapping, and interactive 3D visualization, designed primarily for automated robotic inspection tasks.

## Key Features and Functionalities

### Vision-Based Object Detection
- Real-time object detection and classification using OpenCV.
- Camera calibration functionality ensures accurate spatial measurements.

### LiDAR Integration
- Real-time environmental scanning and continuous generation of 3D point clouds.
- GPU-accelerated processing of LiDAR data to optimize point cloud visualization and analysis.

### Interactive 3D Visualization
- Real-time rendering using OpenGL, supporting interactive inspection through zoom, rotation, and detailed examination.
- User interface developed using ImGui, providing intuitive interactions and control.

### Robotic Control
- Real-time manual control via keyboard input.
- Autonomous scanning mode for automated object inspection tasks.
- Integrated robotic SDK (YMconnect) for precise motion control.

### Data Communication and IPC
- Robust Inter-Process Communication (IPC) mechanisms implemented for seamless integration with Python-based modules, facilitating efficient data transfer between processes.

## System Architecture
- Modular and scalable system architecture designed for maintainability and extensibility.
- Clear separation of concerns between vision, robotic control, data processing, and visualization components.

## Performance Benchmarks (to be added)
- Real-time latency and throughput metrics (vision system, LiDAR processing, visualization pipeline).

## Technologies Used
- **Languages**: Modern C++ (20/23), Python
- **Graphics**: OpenGL, GLFW, GLAD
- **Computer Vision**: OpenCV
- **User Interface**: ImGui
- **LiDAR & Robotics**: Manufacturer-specific LiDAR SDK, YMconnect SDK
- **Communication**: IPC protocols (custom implementation)

## Contributions
As the primary developer, contributions include:
- Design and implementation of core real-time logic and system architecture using modern C++.
- Development of GPU-accelerated point cloud processing and OpenGL visualization.
- Integration and optimization of ImGui interfaces for enhanced user interaction.
- Robust IPC framework ensuring efficient real-time communication with auxiliary Python modules.
- Collaborative project, not primary architect.

## Future Improvements
- Benchmark performance under various hardware conditions.
- Further GPU optimizations to reduce overall latency.
- Expansion of autonomous scanning capabilities with adaptive AI-driven inspection.

## License
Distributed under the MIT License.

## Acknowledgements
- OpenGL for real-time visualization capabilities.
- OpenCV for robust vision processing.
- ImGui for intuitive interface development.
- LiDAR SDK and YMconnect SDK for reliable hardware integration.

## Contact
- **Email**: ngdtuan.dn@gmail.com
- **LinkedIn**: [Nguyen Tuan](https://www.linkedin.com/in/nguyen-tuan-a2a589128/)
- Noted: This project was implemented as part of a collaborative robotics effort. My contributions focused on GPU-based LiDAR visualization and IPC between modules
