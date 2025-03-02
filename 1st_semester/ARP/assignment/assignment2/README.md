# ARP 2nd Assignment

This project extends the drone operation interactive simulator to run across two computers on the same LAN. The processes communicate locally on the same machine, and over a local network (DDS) across machines. In this way, the drone, its controls, and other modules can be split between two networked computers, yet still function as a single coordinated simulator.


## Directory Structure
```
assignment/
├── bin/
│   ├── drone
│   ├── keyboard_manager
│   ├── main
│   ├── map_window
│   ├── obstacle
│   ├── server
│   ├── target
│   ├── ObstaclePub
│   ├── ServerSub
│   ├── TargetPub
│   └── watchdog
├── config/
│   └── config.json
├── images/
│   └── assignment2.pdf
├── include/
│   ├── Objects.idl
│   ├── cJSON.h
│   ├── utils.h
│   └── Generated
│       ├── Objects.hpp
│       ├── ObjectsCdrAux.hpp
│       ├── ObjectsCdrAux.ipp
│       ├── ObjectsPubSubTypes.cxx
│       ├── ObjectsPubSubTypes.hpp
│       ├── ObjectsTypeObjectSupport.cxx
│       └── ObjectsTypeObjectSupport.hpp
├── logs/
│   ├── debug.log
│   └── errors.log
├── src/
│   ├── drone.c
│   ├── keybrd_manager.c
│   ├── main.c
│   ├── map_window.c
│   ├── ObstaclePub.cpp
│   ├── ServerSub.cpp
│   ├── TargetPub.cpp
│   └── watchdog.c
├── .gitignore
├── Makefile
└── README.md
```

- **`bin/`**: Compiled executables.
- **`config/config.json`**: Configuration parameters.
- **`images/assignment2.pdf`**: Architecture sketch.
- **`include/`**: Header files, IDL definitions and sources generated from the IDL.
- **`logs/`**: Log files.
- **`src/`**: Source code for each component.
- **`Makefile`**: Builds all components into `bin/`.


## Architecture Sketch
[Architecture Sketch PDF](images/assignment2.pdf)

- **main.c**: handles process management, inter-process communication, and overall system initialization.
   - Allows Inter-Process Communication. Creates multiple pipes for communication between processes such as the "server", "drone", "obstacles", and "targets", allowing these components to exchange information during execution.
   - Uses the fork() system call to create these processes and execvp() to execute their respective binaries. Those processes include the server, the drone, the obstacles, the targets, and the keyboard manager.
   - A separate watchdog process monitors the state of the child processes to ensure that the system is operating correctly, handling failures if any child process stops unexpectedly.
   - Uses semaphores (sem_open, sem_wait, etc.) to synchronize process startup, ensuring each process begins execution in the correct order.
   - Additionally, it reads a JSON configuration file (config/config.json) using the cJSON library to parse simulation parameters—such as the drone’s initial position/velocity/force and the number of obstacles/targets—and passes them as arguments to the child processes.
   - Logs errors to debug.log and errors.log and exits gracefully if any errors occur during file or pipe creation, process launching, or execution.

   - PRIMITIVES: 
      - fork() is used to create new processes;
      - IPC is handled using pipes;
      - Semaphores are used to synchronize processes with sem_open(), sem_wait(), and sem_close();
      - fopen(), fclose(), and fread() are used for configuration and log file I/O;
      - wait() is used to wait for child processes to terminate;
   - ALGORITHMS:
      - JSON Parsing via cJSON to extract values (e.g., obstacles, targets, drone initial position) from the configuration file.


- **ServerSub.c**: Acts as the central relay for inter-process communication in the system. It functions as a Fast DDS subscriber that listens to two topics—"ObstaclesTopic" and "TargetsTopic"—and forwards the received data via named pipes to the appropriate processes (such as the drone and the map window). In addition to managing Fast DDS subscriptions, the server handles shared memory creation/mapping for the drone state and score, monitors child process IDs via system commands, and periodically signals processes for synchronization and controlled shutdown.
    - PRIMITIVES:
      - Shared Memory: Creates and maps shared memory segments (using shm_open, ftruncate, mmap) for storing the drone state and score
      - Named Pipes & Select(): Uses pipes and the select() system call to perform non-blocking I/O and route data (such as map size updates and key inputs) between processes
      - Fast DDS: Implements Fast DDS subscribers to listen for obstacle and target messages, using DomainParticipant, Subscriber, Topics, and custom DataReaderListeners
      - Signals: Manages process control and watchdog communication using signals (SIGUSR1, SIGUSR2, SIGTERM) and sigaction()
      - Semaphores: Employs semaphores (via sem_open, sem_post, sem_close) for synchronizing the start-up and coordination between processes
      - System Commands: Uses commands executed with popen() (e.g., ps) to retrieve and track child process IDs
      - File I/O: Handles logging for debugging and error messages using fopen(), fprintf(), and fclose().
   - ALGORITHMS:
      - Main Loop & IPC Routing: Implements a main server loop that uses select() to efficiently monitor multiple pipes. When data arrives (for example, updated map size or key presses), the server forwards it to the appropriate process;
      - Child Process Tracking: Retrieves child process IDs by executing system commands and ensures that these IDs are correctly propagated to other components;
      - Watchdog & Shutdown Handling: Handles watchdog requests by responding to SIGUSR1 and, if necessary, initiating a controlled shutdown by sending SIGUSR2, unlinking shared memory segments, and terminating child processes;
      - Shared Memory Management: Establishes and manages shared memory segments for the drone state and score, ensuring that updates are communicated across processes;
      - Thread-Based Signal Generation: Although not directly shown in the DDS subscriber loop, the server coordinates with other processes through periodic signaling and semaphore-based synchronization to trigger events like target or obstacle regeneration.

- **watchdog.c**: Implements a watchdog process that continuously monitors multiple child processes to ensure they remain responsive within a designated timeout period. It periodically sends SIGUSR1 to each monitored process to request a status update. If any process fails to respond within the timeout, the watchdog logs the incident (using debug.log and errors.log) and terminates all processes by sending SIGUSR2 (and, when needed, SIGTERM).
   - PRIMITIVES: 
      - Signals: Uses SIGUSR1 for status requests, SIGUSR2 for initiating termination, and SIGTERM for final shutdown notifications
      - Signal Handling: Configured with sigaction(), sigemptyset(), and sigprocmask() to properly catch and process signals
      - Process Management: Employs kill() to send signals and getpid() to retrieve its own PID
      - Semaphores: Utilizes sem_t, sem_open(), sem_post(), and sem_close() to synchronize the startup of child processes
      - File Operations & Logging: Opens log files using fopen(), writes log messages via a custom LOG_TO_FILE macro, and closes files with fclose()
      - Time Management: Uses time(), difftime(), and strftime() to measure timeouts and timestamp log entries.
   - ALGORITHMS:
      - Timeout Handling: Iteratively sends SIGUSR1 to all monitored processes and waits (using a loop that employs difftime()) for a fixed period to check whether each process has responded;
      - Signal Handling: Processes incoming SIGUSR1 signals to mark processes as responsive; handles SIGUSR2 (and SIGTERM) to initiate a graceful shutdown;
      - Watchdog Loop: In an infinite loop, the watchdog requests status updates, waits for responses, and if any process fails to respond within the timeout period, logs the error, calls kill_processes() to terminate all processes, and exits.

- **drone.c**: defines the functions and data structures related to the drone’s movement and physics interactions (attraction, repulsion, friction, and collision checks). It operates in a loop, responding to map size updates, obstacle/target positions, and keyboard inputs to control the drone’s motion. It updates a shared Score structure when the drone collides with obstacles or reaches targets, and also checks geofences to keep the drone within map boundaries.
   - PRIMITIVES:
      - Shared memory for the drone’s state and the score (shm_open, mmap);
      - Named pipes for receiving map size, obstacles, and targets (select() multiplexing);
      - Signals (sigaction) for handling watchdog shutdown or status checks;
      - A dedicated thread (pthread_create(), pthread_join()) to update the drone’s position continuously (update_drone_position_thread());
      - Dynamic memory allocation (malloc(), free()) for obstacles and targets;
      - Logging of errors and debug information.
   - ALGORITHMS:
      - A main loop using select() to read incoming data (map size, obstacles, targets) and keyboard input;
      - Parsing and storing obstacle/target data, then applying physical forces (repulsion/attraction) and friction in a background thread;
      - Handling collisions with obstacles and targets, updating the shared score accordingly;
      - Checking geofences to keep the drone within bounds;
      - Using signal handlers to manage shutdown signals from the watchdog and to synchronize with other processes.

- **keyboard_manager.c**: This component is responsible for capturing user keyboard input and updating the interactive UI using ncurses. It sends key press events through a pipe to the server process for controlling the drone. In addition, it continuously updates a separate information window (displaying drone dynamics such as position, velocity, and force) via a dedicated thread, handles terminal resize events, and processes shutdown signals. Moreover, when the user requests a restart (by pressing 'O'), it calls a restart script ("restart.sh") to reset the simulation.
   - PRIMITIVES:
      - The game structure stores the terminal (map) dimensions as well as the number of obstacles and targets;
      - Shared memory is used to access the current drone state and score;
      - Pipes facilitate IPC by sending the terminal dimensions to the server and receiving updated obstacle and target data;
      - ncurses is used for drawing the map—including the outer border, drone, obstacles (rendered as "O"), and targets (rendered as "T" along with their identifying numbers);
      - Semaphores are employed for synchronizing process execution;
      - Signal handling is implemented to manage terminal resize events (SIGWINCH) and shutdown commands (SIGUSR2).
   - ALGORITHMS:
      - Initializes logging, ncurses, shared memory regions, and semaphores;
      - Continuously updates the UI by:
         - Drawing an outer border that displays the current score;
         - Rendering the drone, obstacles, and targets based on their latest positions;
         - Using select() to multiplex input from pipes that carry updates for obstacles and targets;
      - Handles terminal resizing by recalculating the window dimensions, updating the game structure, notifying the server of the new dimensions, and redrawing the interface
      - Cleans up resources gracefully upon receiving shutdown signals

- **map_window.c**: This component is responsible for rendering the simulation environment in the terminal using ncurses. It displays the current state of the simulation—including the drone, obstacles, and targets—by retrieving the drone’s position and score from shared memory. It also handles dynamic terminal resizing and communicates the updated window dimensions to the server via a pipe. Additionally, it synchronizes its startup with other processes using semaphores and manages graceful shutdown through signal handling.
   - PRIMITIVES:
      - "game" structure (map dimensions, shared memory pointer for drone state and score, number of obstacles and targets);
      - Display functions;
      - IPC communication;
      - Signal handling.
   - ALGORITHMS:
      - Initialzation of log files, ncurses, shared memory and semaphores;
      - Update the data of drone, obstacles and targets;
      - Resizing of display windows and cleanup.

- **ObstaclePub.cpp**:Generates and manages obstacles in the drone’s environment using Fast DDS. This component creates a DomainParticipant, registers its message type, and then creates a topic, publisher, and data writer to continuously publish randomly generated obstacle data based on the provided map dimensions and number of obstacles. It also uses a custom DataWriterListener (with semaphore synchronization) to ensure that the publisher only starts publishing once a matching subscriber is available. The publisher periodically updates obstacle positions and communicates with the watchdog via signals for controlled shutdown.
   - PRIMITIVES:
      - DDS Entities: Uses Fast DDS primitives (DomainParticipant, Publisher, Topic, DataWriter, and TypeSupport) to handle obstacle publishing;
      - Shared Memory: Synchronizes game state (e.g. map dimensions) via command-line configuration parameters;
      - Semaphores: Utilizes sem_open() and sem_post() to synchronize the publisher with other processes (via a sync semaphore);
      - Signals: Handles SIGUSR1 and SIGUSR2 (via sigaction) to interact with the watchdog process and to trigger graceful shutdown;
      - Dynamic Memory & Logging: Employs dynamic memory allocation (for the obstacle message) and standard file operations (fopen(), fprintf()) for logging events and errors.
   - ALGORITHMS:
      - Initialization & Setup: Creates a DomainParticipant on a specified domain, registers the obstacle type, and sets up a topic, publisher, and DataWriter with a custom listener;
      - Synchronization: Uses a custom DataWriterListener to detect when a subscriber has matched. This listener waits on a semaphore and then signals readiness to publish (by setting a global flag);
      - Obstacle Generation & Publication: Implements a loop that, once the subscriber is ready, generates random obstacle positions based on the map width and height and publishes the obstacle data every 15 seconds;
      - Signal-Driven Shutdown: Uses signal handlers to receive shutdown signals (SIGUSR2) from the watchdog, ensuring that the publisher logs the event, cleans up DDS entities, and exits gracefully.

- **TargetPub.c**: Generates and manages targets in the drone’s environment using Fast DDS. This component creates a DomainParticipant, registers the target type, and then creates a topic, publisher, and data writer to continuously publish randomly generated target data based on the provided map dimensions and number of targets. It uses a custom DataWriterListener (with semaphore‐based synchronization) to ensure that publication only begins once a matching subscriber is available. The publisher periodically regenerates target positions and communicates with the watchdog via signals to enable controlled shutdown.
   - PRIMITIVES:
      - DDS Entities: Utilizes Fast DDS objects (DomainParticipant, Publisher, Topic, DataWriter, and TypeSupport) to publish target data;
      - Shared Memory & IPC: Configuration parameters (map width, height, and number of targets) are provided via command-line arguments;
      - Semaphores: Uses sem_open() and sem_post() for synchronization between publishers;
      - Signals: Employs sigaction() for SIGUSR1, SIGUSR2 (and SIGTERM) to handle inter-process communication with the watchdog and trigger graceful shutdown;
      - Dynamic Memory & Logging: Implements dynamic memory for target data (via Fast DDS internal data structures) and standard file I/O (fopen(), fprintf()) for logging.
   - ALGORITHMS:
      - Initialization & Setup: Creates a DomainParticipant on a specified domain, registers the target type, and sets up the topic, publisher, and DataWriter along with a custom listener;
      - Synchronization: The custom DataWriterListener signals (via a synchronization semaphore) when a subscriber is matched, allowing the publisher to start generating and publishing target data;
      - Target Generation & Publication: Implements a loop that, once synchronization is complete, generates random target positions within the map constraints and publishes the target data every 15 seconds;
      - Signal-Driven Shutdown: Uses signal handlers to receive shutdown signals (SIGUSR2) from the watchdog, ensuring that the publisher logs the event, cleans up DDS entities, and exits gracefully.


## Prerequisites
- **Konsole**: Ensure that Konsole is installed on your system and install wmctrl.
   ```bash
   sudo apt install konsole
   sudo apt install wmctrl
   ```
- **NCurses**: Ensure that NCurses is installed on your system.
   ```bash
   sudo apt install libncurses-dev
   ```
- **cJSON library**: Ensure that cJSON is installed on your system.
   ```bash
   sudo apt install libcjson-dev
   ```
- **libbsd library**: Ensure that libbsd is installed on your system.
   ```bash
   sudo apt install libbsd-dev
   ```


## Building
1. Ensure you have `gcc`, `g++` and `make` installed.
2. In the `assignment2/` directory, run:
   ```bash
   make
   ```
    This will compile all .c and .cpp files under src/ into executables under bin/.


## Running
Run :
```bash
./bin/main
```

## Operational instructions
- After running, press `s` to start the simulation or `q` to quit.
- When starting the simulation, be sure to click on the window of `Input Display`, then command the drone with your keyboard.
```
\  ^  /                                       E  R  T
<  F  >          with the following keys       D  F  G
/  v  \                                         C  V  B
```
- Press the keys around the letter `F` to increase the force of command of the drone in this direction (`e/r/t/g/b/v/c/d`). _Compatible for keyboards of type QWERTY, AZERTY and QWERTZ._ The drone will be represented by an orange cross (`"+"`).
- Press `F` to remove command forces.
- Press `Q` to quit.
- Press `O` to start over the simulation.

- Obstacles (red `"O"`) and geofences on the borders will repulse the drone.
- Targets (green _`"number"`_) will attract the drone.
- Score: 
   - Hit a random target and win 1 point;
   - Hit the target such that _index_(target) = 1 + _index_(last target hit) and win _index_ points;
   - Hit an obstacle and lose half as much points as the index of the last target you hit. You will lose your sequence. 