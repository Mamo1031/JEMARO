# ARP 1st Assignment

This project is a drone operation interactive simulator implemented with multiple processes communicating by exchanging data through shared memory or pipes, and they all operate as child processes of a central main process.


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
│   └── watchdog
├── config/
│   └── config.json
├── images/
│   └── assignment1.pdf
├── include/
│   ├── cJSON.h
│   └── utils.h
├── logs/
│   ├── debug.log
│   └── errors.log
├── src/
│   ├── drone.c
│   ├── keybrd_manager.c
│   ├── main.c
│   ├── map_window.c
│   ├── obstacles.c
│   ├── server.c
│   ├── target.c
│   └── watchdog.c
├── .gitignore
├── Makefile
└── README.md
```

- **`bin/`**: Compiled executables.
- **`config/config.json`**: Configuration parameters.
- **`images/assignment1.pdf`**: Architecture sketch.
- **`include/`**: Header files 
- **`logs/`**: Log files.
- **`src/`**: Source code for each component.
- **`Makefile`**: Builds all components into `bin/`.


## Architecture Sketch
[Architecture Sketch PDF](images/assignment1.pdf)

- **main.c**: handles process management, inter-process communication, and overall system initialization.
   - Allows Inter-Process Communication. Creates multiple pipes for communication between processes that are the "server", "drone", "obstacles", and "targets". This allows the components to exchange information during execution.
   - Uses the fork() system call to create these processes and execvp() to execute their respective binaries. Those processes are the server, the drone, the obstacles, the targets and the keyboard manager.
   - A separate process that monitors the state of the child processes is created to ensure that the system is operating correctly and handles failures if any child process stops unexpectedly.
   - The component uses semaphores to synchronize process execution, ensuring that each process starts in the correct order.
   - Logs errors to debug.log and errors.log and exits gracefully if any errors occur during file or pipe creation, process launching, or execution.

   - PRIMITIVES: 
      - fork() is used to create new processes;
      - IPC is handled using pipes;
      - Semaphores are used to synchronize the process with sem_open(), sem_wait(), and sem_close();
      - fopen(), fclose(), and fread() are used to handle file input/output operations for configuration and logs;
      - wait() is used to wait for child processes to terminate;
   - ALGORITHMS:
      - JSON Parsing: to extract values from a configuration file for simulation parameters (e.g., obstacles, targets, initial drone positions).


- **server.c**: central relay for inter-process communication, managing data flow between the drone, map, input, obstacle, and target processes. It also handles shared memory, process monitoring, and periodic signaling to ensure synchronization and control.
    - PRIMITIVES:
      - Shared memory for storing drone state and score (shm_open, mmap);
      - Named pipes for inter-process communication (select() for non-blocking I/O);
      - Signals for process control and watchdog communication (SIGUSR1, SIGUSR2, SIGTERM);
      - Semaphores for process synchronization (sem_open, sem_post);
      - Multithreading for periodic signal generation (pthread_create);
      - System commands for process management (ps aux, grep, popen);
      - File handling for logging and debugging (fopen(), fprintf()).
   - ALGORITHMS:
      - Main server loop using select() to efficiently monitor multiple pipes for incoming data and forward it to the correct process;
      - Finding and tracking child process IDs using system commands;
      - Handle watchdog requests, shutdown sequences, and controlled termination of processes with signals;
      - Shared memory management to create and map memory segments for drone state and score tracking;
      - Thread-based signal generation that periodically sends SIGTERM to target and obstacle processes to trigger regeneration.

- **watchdog.c**: implements a watchdog process that monitors multiple child processes, ensuring they are responsive within a specified timeout period with SIGUSR1. If any monitored process fails to respond, the watchdog takes action by logging the issue in debug.log and errors.log and terminating all processes with SIGUSR2 or SIGTERM.
   - PRIMITIVES: 
      - For signals: SIGUSR1, SIGUSR2, SIGTERM;
      - For signal handling: sigaction(), sigemptyset(), sigpromask();
      - For process management: kill(), getpid();
      - For semaphores: sem_t, sem_open(), sem_post(), sem_close();
      - For file operations: fopen(), fgets(), fclose();
      For time management: time(), difftime(), strftime();
   - ALGORITHMS:
      - Timeout Handling algorithm
      - Signal Handling Algorithm
      - Watchdog Loop

- **drone.c**: defines the functions and structures related to the drone's movement and physics interactions (forces like attraction, repulsion and friction). It operates in a loop, responding to changes in the environment, such as map size, obstacle positions, and target positions, while also processing keyboard inputs to control the drone. It handles object collisions (obstacles and targets), updating the drone’s position, and responding to key presses.
   - PRIMITIVES:
      - Shared memory for the drone state and the score (shm_open, mmap);
      - Named pipes to receive map size, obstacles and targets from external processes (select());
      - Signals to change the behaviour of the drone (sigaction);
      - Threads to update the drone's position position (pthread_create() and update_drone_position_thread());
      - Memory allocation for obstacles and targets (malloc() and free());
      - Logging errors.
   - ALGORITHMS:
      - Main loop using select() for multiplexing;
      - Parsing incoming data for obstacles and targets and updating the simulation state;
      - Update of the drone position in a separate thread;
      - Handling asynchronous events with signal handlers;
      - Managing dynamic memory for obstacles and targets.

- **keyboard_manager.c**: captures user input, updates UI elements, and communicates with other processes via shared memory, pipes, and signals. Also calls the "restart" executable generated by the main if the user asks to.
   - PRIMITIVES:
      - ncurses for UI;
      - pthread for multithreading;
      - shm for IPC;
      - signals for control.
   - ALGORITHMS:
      - Reading keyboard input;
      - Creating and filling UI windows for visual feedback;
      - Update the drone position;
      - Signal handling.

- **map_window.c**: displays the simulation environment with targets, obstacles and the drone in the terminal using ncurses, while handling dynamic window resizing and real-time updates of the positions. It retrieves the drone's position and current score from shared memory and handles synchronization and shutdown with semaphores.
   - PRIMITIVES:
      - "game" structure (map dimensions, shared memory pointer for drone state and score, number of obstacles and targets);
      - Display functions;
      - IPC communication;
      - Signal handling.
   - ALGORITHMS:
      - Initialzation of log files, ncurses, shared memory and semaphores;
      - Update the data of drone, obstacles and targets;
      - Resizing of display windows and cleanup.

- **obstacle.c**: generates and manages obstacles in the drone's environment. It receives information about the map size and dynamically adjusts obstacle positions. It communicates with other processes via pipes and signals, to respond to updates of the environment.
   - PRIMITIVES:
      - Shared memory to synchronize game state (shm_open, mmap);
      - Named pipes to receive map size and send obstacle positions (select());
      - Signals to handle process interruptions and state changes (sigaction for SIGUSR1, SIGUSR2, SIGTERM);
      - Semaphores for child process synchronization (sem_open, sem_post);
      - Dynamic memory allocation for obstacle management (malloc(), free());
      - File handling for logging errors and events (fopen(), fprintf()).
   - ALGORITHMS:
      - Main loop with select() to listen for map updates;
      - Random obstacle generation based on map size constraints;
      - Signal-driven behavior to respond to system events;
      - Inter-process communication via pipes to update obstacle positions dynamically;
      - Watchdog communication to ensure process monitoring and controlled shutdown.

- **target.c**: generates and manages targets in the drone's environment. It dynamically places targets on the map, listens for updates, and handles communication with other processes.
   - PRIMITIVES:
      - Shared memory for game state synchronization (shm_open, mmap);
      - Named pipes to receive map size and send target positions (select());
      - Signals for handling process interruptions and state changes (sigaction for SIGUSR1, SIGUSR2, SIGTERM);
      - Semaphores for process synchronization (sem_open, sem_post);
      - Dynamic memory allocation for storing target data (malloc(), free());
      - File handling for debugging and logging errors (fopen(), fprintf()).
   - ALGORITHMS:
      - Main loop with select() to monitor map size updates;
      - Random target placement within defined map constraints;
      - Regenerate targets and communicate with the watchdog with signals;
      - Inter-process communication via pipes to update target positions dynamically;
      - Watchdog monitoring to ensure controlled shutdown.


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
1. Ensure you have `gcc` and `make` installed.
2. In the `assignment1/` directory, run:
   ```bash
   make
   ```
    This will compile all .c files under src/ into executables under bin/.


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