# Robotic Arm Control using Unity VR and Python

## Introduction

This repository hosts the codebase for controlling a physical robotic arm via a Unity VR application, facilitated by a Flask server. The system integrates virtual reality technology provided by the Meta Quest 3 headset to manipulate a virtual twin of the real robotic arm, transmitting joint positions to a Flask server which, in turn, translates these positions into commands for the physical arm.

## System Architecture

The system architecture is composed of three main components:

1. **Unity VR Application**: The Unity VR application serves as the interface for users to interact with the robotic arm. It simulates the movement of the arm in a virtual environment, allowing users to manipulate it using hand controllers provided by the Meta Quest 3 headset.

2. **Flask Server**: A Flask server acts as the intermediary between the Unity VR application and the physical robotic arm. It receives joint position data from the VR application and processes it to generate commands for controlling the real arm.

3. **Serial Control To PCB**: USB Serial is responsible for managing the communication between the Unity VR application, the Flask server, and the physical robotic arm. PCB firmware is handling data transmission, protocol conversion, and command execution.

## Repository Structure
- `app.py`: Includes the Flask server implementation for receiving and processing joint position data.

## Usage

To utilize this system effectively, follow these steps:

1. Clone this repository to your local machine.
2. Set up and configure the Unity VR application on the Meta Quest 3 headset.
3. Deploy and run the Flask server on a machine accessible to both the VR headset and the physical robotic arm. ( Make sure to run in terminal --> flask run --host=0.0.0.0)
4. Flash PCB firmware according to the specifications and communication protocols of your robotic arm.
5. Initiate the Unity VR application and manipulate the virtual twin of the robotic arm to control its physical counterpart.

## Dependencies

- Unity 3D
- Python 3
- Flask

## Contribution Guidelines

Contributions to this project are encouraged and appreciated. To contribute, please follow these guidelines:

- Fork the repository and create a new branch for your contributions.
- Make your changes and ensure that they adhere to the project's coding standards.
- Submit a pull request detailing the changes you've made and their significance.

## License

This project is licensed under the [MIT License](LICENSE).

