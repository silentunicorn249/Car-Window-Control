## Car Window Control
This project implements a car window control system using an ARM Cortex-M microcontroller (TM4C123GH6PM) and FreeRTOS. It provides functionalities for controlling the passenger's window with both manual and automatic modes, along with safety features such as limit switches, jam protection, and window lock.

## Project Checklist
Two panels are installed to control the passenger’s window.
FreeRTOS is used in the project implementation.
Implementation of 2 limit switches to limit the window motor from top and bottom limits of the window.
Continuous operation: When the power window switch is pushed or pulled continuously, the window opens or closes until the switch is released.
Short operation: When the power window switch is pushed or pulled shortly, the window fully opens or closes.
Window lock: When the window lock switch is turned on, the opening and closing of all windows except the driver’s window are disabled.
Jam Protection is implemented.
Queues are used in the project implementation.
Semaphores/Mutex are used in the project implementation.
Project documentation is provided.

## Function implementations
The main.c file contains the implementation of the car window control system. It initializes the system, sets up interrupt handlers, defines tasks for passenger and driver window controls, and manages motor operations along with safety features.

## Usage
To use the car window control system, follow these steps:

Ensure all necessary hardware components are properly connected.
Flash the provided firmware onto the microcontroller.
Power on the system and interact with the control panels to operate the windows.
Refer to the project documentation for detailed usage instructions and safety guidelines.
## Project Structure
main.c: Contains the main implementation of the car window control system.
DIO.h: Header file for digital input/output operations.
queue.h: Header file for queue data structure.
timers.h: Header file for timer operations.
semphr.h: Header file for semaphore operations.
## Contributing
Contributions to the project are welcome. If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

## License
This project is licensed under the MIT License. Feel free to use, modify, and distribute it as per the terms of the license.
