# ARP assignments 1 and 2

The project consists in a drone operation interactive simulator.

## Assignment 1: One computer
[assignment1](assignment1)
First assignment provides a simulator using ncurses, where the drone, controlled via the keyboard, must reach targets while avoiding obstacles that generate a repulsive force. It follows a realistic dynamic model with inertia and friction. The program is structured into multiple processes communicating via pipes/signals, including a central server, a keyboard manager, a physics engine, and a monitoring system. A final score evaluates performance based on time, targets reached, and obstacles avoided.

## Assignment 2: Two computers - FAST DDS
[assignment2](assignment2)
The second assignment extends the simulator to two computers on the same LAN, both running identical programs. They communicate via DDS and can swap roles: one controls the drone while the other generates targets and obstacles. Each computer maintains the full architecture from the first assignment, but only some processes are active at a time. The watchdog, logs, and parameter files remain as specified. The programs publish and subscribe to data, ensuring real-time synchronization between the two systems.