/**
 * ServerSub.cpp
 * 
 * This program implements a Fast DDS subscriber that listens to two topics: 
 * "ObstaclesTopic" and "TargetsTopic". When data is available from either topic,
 * it processes the received sample and writes the formatted data to the appropriate 
 * inter-process communication pipes.
 */

 #include <chrono>
 #include <thread>
 #include <iostream>
 #include <vector>
 #include <string>
 #include <atomic>
 #include <cstring>
 
 #include <fastdds/dds/domain/DomainParticipant.hpp>
 #include <fastdds/dds/domain/DomainParticipantFactory.hpp>
 #include <fastdds/dds/subscriber/DataReader.hpp>
 #include <fastdds/dds/subscriber/DataReaderListener.hpp>
 #include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
 #include <fastdds/dds/subscriber/SampleInfo.hpp>
 #include <fastdds/dds/subscriber/Subscriber.hpp>
 #include <fastdds/dds/topic/TypeSupport.hpp>
 
 #include "utils.h"
 #include "Generated/ObjectsPubSubTypes.hpp"
 #include <signal.h>
 #include <sys/mman.h>
 #include <sys/select.h>
 #include <unistd.h>
 
 using namespace eprosima::fastdds::dds;
 
 // Global variables for logging, process IDs, shared memory, and IPC file descriptors
 FILE* debugLog, *errorLog;       // Log file pointers
 pid_t watchdogPID, mapWindowPID;
 Drone* sharedDrone;
 float* sharedScore;
 
 // IPC pipe file descriptors for writing to drone and map processes
 int pipeDroneWriteSizeFD, pipeDroneWriteKeyFD;
 int pipeDroneWriteObstaclesFD, pipeDroneWriteTargetsFD;
 int pipeMapWriteObstaclesFD, pipeMapWriteTargetsFD;
 
 //------------------------------------------------------------------------------
 // ServerSub Class
 //------------------------------------------------------------------------------
 
 class ServerSubscriberApp
 {
 private:
     DomainParticipant* participant_;
     Subscriber* subscriber_;
     DataReader* dataReaderObstacles_;
     DataReader* dataReaderTargets_;
     Topic* topicObstacles_;
     Topic* topicTargets_;
     TypeSupport typeSupport_;
 
     // Custom DataReaderListener class to handle subscription events and data reception
     class CustomDataReaderListener : public DataReaderListener
     {
     public:
         CustomDataReaderListener() : sampleCount_(0) {}
         ~CustomDataReaderListener() override {}
 
         // Called when the subscription matches with a publisher
         void on_subscription_matched(DataReader* /*reader*/, const SubscriptionMatchedStatus& info) override
         {
             if (info.current_count_change == 1)
             {
                 std::cout << "Subscriber matched." << std::endl;
             }
             else if (info.current_count_change == -1)
             {
                 std::cout << "Subscriber unmatched." << std::endl;
             }
             else
             {
                 std::cout << info.current_count_change
                           << " is not a valid value for subscription match change." << std::endl;
             }
         }
 
         // Called when new data is available
         void on_data_available(DataReader* reader) override
         {
             SampleInfo sampleInfo;
             // Get the topic name for routing purposes
             auto topicName = reader->get_topicdescription()->get_name();
 
             // Pointer to the received message and a string to track the topic name
             Objects* receivedMessage = nullptr;
             std::string currentTopic;
 
             if (topicName == "ObstaclesTopic")
             {
                 receivedMessage = &obstacleMessage_;
                 currentTopic = "ObstaclesTopic";
             }
             else if (topicName == "TargetsTopic")
             {
                 receivedMessage = &targetMessage_;
                 currentTopic = "TargetsTopic";
             }
 
             // If a valid message is received, process it
             if (receivedMessage != nullptr && reader->take_next_sample(receivedMessage, &sampleInfo) == RETCODE_OK)
             {
                 if (sampleInfo.valid_data)
                 {
                     std::cout << "Received data from " << currentTopic << std::endl;
 
                     // Format the received data into a string for further IPC
                     std::vector<Object> objectList(receivedMessage->objects_number());
                     std::string formattedData = std::to_string(receivedMessage->objects_number()) + ":";
                     for (int i = 0; i < receivedMessage->objects_number(); i++)
                     {
                         objectList[i] = { receivedMessage->x()[i], receivedMessage->y()[i],
                                           (currentTopic == "ObstaclesTopic" ? 'o' : 't'), false };
 
                         formattedData += std::to_string(objectList[i].pos_x) + "," +
                                          std::to_string(objectList[i].pos_y) + "," +
                                          objectList[i].type + "," +
                                          std::to_string(objectList[i].hit ? 1 : 0);
                         if (i + 1 < receivedMessage->objects_number())
                         {
                             formattedData += "|";
                         }
                     }
 
                     // Write the formatted data to the appropriate pipes
                     if (currentTopic == "ObstaclesTopic")
                     {
                         write(pipeDroneWriteObstaclesFD, formattedData.c_str(), formattedData.size());
                         write(pipeMapWriteObstaclesFD, formattedData.c_str(), formattedData.size());
                     }
                     else if (currentTopic == "TargetsTopic")
                     {
                         write(pipeDroneWriteTargetsFD, formattedData.c_str(), formattedData.size());
                         write(pipeMapWriteTargetsFD, formattedData.c_str(), formattedData.size());
                     }
                 }
             }
         }
 
         // Member variables to store messages for each topic
         Objects obstacleMessage_, targetMessage_;
 
         // Count of samples processed
         std::atomic_int sampleCount_;
     } customListener_;
 
 public:
     // Constructor: initializes member pointers and creates a new TypeSupport instance for Objects
     ServerSubscriberApp()
         : participant_(nullptr)
         , subscriber_(nullptr)
         , dataReaderObstacles_(nullptr)
         , dataReaderTargets_(nullptr)
         , topicObstacles_(nullptr)
         , topicTargets_(nullptr)
         , typeSupport_(new ObjectsPubSubType())
     {
     }
 
     // Destructor: cleans up allocated resources
     virtual ~ServerSubscriberApp()
     {
         if (dataReaderObstacles_ != nullptr)
         {
             subscriber_->delete_datareader(dataReaderObstacles_);
         }
         if (dataReaderTargets_ != nullptr)
         {
             subscriber_->delete_datareader(dataReaderTargets_);
         }
         if (topicObstacles_ != nullptr)
         {
             participant_->delete_topic(topicObstacles_);
         }
         if (topicTargets_ != nullptr)
         {
             participant_->delete_topic(topicTargets_);
         }
         if (subscriber_ != nullptr)
         {
             participant_->delete_subscriber(subscriber_);
         }
         DomainParticipantFactory::get_instance()->delete_participant(participant_);
     }
 
     // Initialize the subscriber by creating the DomainParticipant, registering the type, and creating topics and DataReaders
     bool init()
     {
         DomainParticipantQos participantQos;
         participantQos.name("Participant_subscriber");
 
         // Create the DomainParticipant
         participant_ = DomainParticipantFactory::get_instance()->create_participant(1, participantQos);
         if (participant_ == nullptr)
         {
             return false;
         }
 
         // Register the type support for Objects
         typeSupport_.register_type(participant_);
 
         // Create topics for obstacles and targets
         topicObstacles_ = participant_->create_topic("ObstaclesTopic", typeSupport_.get_type_name(), TOPIC_QOS_DEFAULT);
         topicTargets_ = participant_->create_topic("TargetsTopic", typeSupport_.get_type_name(), TOPIC_QOS_DEFAULT);
         if (topicObstacles_ == nullptr || topicTargets_ == nullptr)
         {
             return false;
         }
 
         // Create the Subscriber
         subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
         if (subscriber_ == nullptr)
         {
             return false;
         }
 
         // Create DataReaders for both topics using the custom listener
         dataReaderObstacles_ = subscriber_->create_datareader(topicObstacles_, DATAREADER_QOS_DEFAULT, &customListener_);
         dataReaderTargets_ = subscriber_->create_datareader(topicTargets_, DATAREADER_QOS_DEFAULT, &customListener_);
         if (dataReaderObstacles_ == nullptr || dataReaderTargets_ == nullptr)
         {
             return false;
         }
 
         return true;
     }
 
     // Run the subscriber loop: uses select() to monitor pipes and forward data accordingly
     void run(int pipeDroneWriteSizeFD, int pipeDroneWriteKeyFD, int pipeInputReadKeyFD, int pipeMapReadSizeFD)
     {
         char buffer[4096];
         fd_set readFDs;
         struct timeval timeout;
         int maxFD = -1;
         if (pipeMapReadSizeFD > maxFD)
         {
             maxFD = pipeMapReadSizeFD;
         }
         if (pipeInputReadKeyFD > maxFD)
         {
             maxFD = pipeInputReadKeyFD;
         }
 
         // Continuously monitor the input pipes for incoming data
         while (true)
         {
             FD_ZERO(&readFDs);
             FD_SET(pipeInputReadKeyFD, &readFDs);
             FD_SET(pipeMapReadSizeFD, &readFDs);
 
             timeout.tv_sec = 1;
             timeout.tv_usec = 0;
 
             int activity;
             do {
                 activity = select(maxFD + 1, &readFDs, NULL, NULL, &timeout);
             } while (activity == -1 && errno == EINTR);
 
             if (activity < 0)
             {
                 perror("[SERVER]: Error in select");
                 LOG_TO_FILE(errorLog, "Error in select on pipe reads");
                 break;
             }
             else if (activity > 0)
             {
                 memset(buffer, '\0', sizeof(buffer));
                 // If the map size data is received from the map process, forward it to the drone
                 if (FD_ISSET(pipeMapReadSizeFD, &readFDs))
                 {
                     ssize_t bytesRead = read(pipeMapReadSizeFD, buffer, sizeof(buffer) - 1);
                     if (bytesRead > 0)
                     {
                         buffer[bytesRead] = '\0';
                         write(pipeDroneWriteSizeFD, buffer, strlen(buffer));
                     }
                 }
                 // If a key input is received, forward it to the drone
                 if (FD_ISSET(pipeInputReadKeyFD, &readFDs))
                 {
                     ssize_t bytesRead = read(pipeInputReadKeyFD, buffer, sizeof(buffer) - 1);
                     if (bytesRead > 0)
                     {
                         buffer[bytesRead] = '\0';
                         write(pipeDroneWriteKeyFD, buffer, strlen(buffer));
                     }
                 }
             }
         }
         // Close the pipe file descriptors before exiting
         close(pipeDroneWriteSizeFD);
         close(pipeDroneWriteKeyFD);
         close(pipeMapReadSizeFD);
         close(pipeInputReadKeyFD);
     }
 };
 
 //------------------------------------------------------------------------------
 // Helper function to retrieve the PID of a child process running under a terminal (e.g., Konsole)
 //------------------------------------------------------------------------------
 int getTerminalChildPID(pid_t parentPID)
 {
     char command[100];
     // Build command to list the child PIDs of the specified parent process
     sprintf(command, "ps --ppid %d -o pid= 2>/dev/null", parentPID);
     FILE* pipe = popen(command, "r");
     if (pipe == nullptr)
     {
         perror("[SERVER]: Error opening pipe to retrieve terminal child PID");
         LOG_TO_FILE(errorLog, "Error opening pipe to retrieve terminal child PID");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     int childPID;
     fscanf(pipe, "%d", &childPID);
     pclose(pipe);
     return childPID;
 }
 
 //------------------------------------------------------------------------------
 // Signal handler for SIGUSR1 and SIGUSR2 signals
 //------------------------------------------------------------------------------
 void signalHandler(int sig, siginfo_t* info, void* context)
 {
     (void)context; // Unused parameter
     if (sig == SIGUSR1)
     {
         watchdogPID = info->si_pid;
         LOG_TO_FILE(debugLog, "Signal SIGUSR1 received from WATCHDOG");
         kill(watchdogPID, SIGUSR1);
     }
     if (sig == SIGUSR2)
     {
         LOG_TO_FILE(debugLog, "Shutting down by the WATCHDOG");
         std::cout << "Server shutting down by the WATCHDOG: " << getpid() << std::endl;
         // Send SIGUSR2 to the map window process to instruct shutdown
         if (kill(mapWindowPID, SIGUSR2) == -1)
         {
             perror("[SERVER]: Error sending SIGTERM signal to the MAP");
             LOG_TO_FILE(errorLog, "Error sending SIGTERM signal to the MAP");
             exit(EXIT_FAILURE);
         }
 
         // Unlink shared memory segments
         if (shm_unlink(DRONE_SHARED_MEMORY) == -1)
         {
             perror("[SERVER]: Error unlinking the drone shared memory");
             LOG_TO_FILE(errorLog, "Error unlinking the drone shared memory");
             fclose(debugLog);
             fclose(errorLog);
             exit(EXIT_FAILURE);
         }
         if (shm_unlink(SCORE_SHARED_MEMORY) == -1)
         {
             perror("[SERVER]: Error unlinking the score shared memory");
             LOG_TO_FILE(errorLog, "Error unlinking the score shared memory");
             fclose(debugLog);
             fclose(errorLog);
         }
 
         // Unlink semaphores
         sem_unlink("drone_sem");
         sem_unlink("/map_sem");
 
         fclose(errorLog);
         fclose(debugLog);
         exit(EXIT_SUCCESS);
     }
 }
 
 //------------------------------------------------------------------------------
 // Function to create and map shared memory for the Drone structure
 //------------------------------------------------------------------------------
 int createDroneSharedMemory()
 {
     int shm_fd = shm_open(DRONE_SHARED_MEMORY, O_CREAT | O_RDWR, 0666);
     if (shm_fd == -1)
     {
         perror("[SERVER]: Error opening the drone shared memory");
         LOG_TO_FILE(errorLog, "Error opening the drone shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     
     // Set the shared memory size to the size of the Drone structure
     if (ftruncate(shm_fd, sizeof(Drone)) == -1)
     {
         perror("[SERVER]: Error setting the size of the drone shared memory");
         LOG_TO_FILE(errorLog, "Error setting the size of the drone shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     
     // Map the shared memory to the global drone pointer
     sharedDrone = (Drone*)mmap(0, sizeof(Drone), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
     if (sharedDrone == MAP_FAILED)
     {
         perror("[SERVER]: Error mapping the drone shared memory");
         LOG_TO_FILE(errorLog, "Error mapping the drone shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     LOG_TO_FILE(debugLog, "Created and opened the drone shared memory");
     return shm_fd;
 }
 
 //------------------------------------------------------------------------------
 // Function to create and map shared memory for the score (float)
 //------------------------------------------------------------------------------
 int createScoreSharedMemory()
 {
     int shm_fd = shm_open(SCORE_SHARED_MEMORY, O_CREAT | O_RDWR, 0666);
     if (shm_fd == -1)
     {
         perror("[SERVER]: Error opening the score shared memory");
         LOG_TO_FILE(errorLog, "Error opening the score shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     
     // Set the shared memory size to the size of a float
     if (ftruncate(shm_fd, sizeof(float)) == -1)
     {
         perror("[SERVER]: Error setting the size of the score shared memory");
         LOG_TO_FILE(errorLog, "Error setting the size of the score shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     
     // Map the shared memory to the global score pointer
     sharedScore = (float*)mmap(0, sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
     if (sharedScore == MAP_FAILED)
     {
         perror("[SERVER]: Error mapping the score shared memory");
         LOG_TO_FILE(errorLog, "Error mapping the score shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     *sharedScore = 0;
     LOG_TO_FILE(debugLog, "Created and opened the score shared memory");
     return shm_fd;
 }
 
 int main(int argc, char* argv[])
 {
     std::cout << "Starting Server subscriber" << std::endl;
 
     // Create a new instance of the ServerSubscriberApp
     ServerSubscriberApp* serverApp = new ServerSubscriberApp();
 
     // Open log files for debugging and error logging
     debugLog = fopen("debug.log", "a");
     if (debugLog == NULL)
     {
         perror("[SERVER]: Error opening the debug log file");
         exit(EXIT_FAILURE);
     }
     errorLog = fopen("errors.log", "a");
     if (errorLog == NULL)
     {
         perror("[SERVER]: Error opening the error log file");
         exit(EXIT_FAILURE);
     }
 
     // Check for the required number of command-line parameters
     if (argc < 8)
     {
         LOG_TO_FILE(errorLog, "Invalid number of parameters");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
 
     LOG_TO_FILE(debugLog, "Process started");
 
     // Open the semaphore for child process synchronization
     sem_t* execSemaphore = sem_open("/exec_semaphore", 0);
     if (execSemaphore == SEM_FAILED)
     {
         LOG_TO_FILE(errorLog, "Failed to open the exec semaphore");
         perror("[SERVER]: Failed to open the exec semaphore");
         exit(EXIT_FAILURE);
     }
     sem_post(execSemaphore);
     sem_close(execSemaphore);
 
     // Set up inter-process communication pipes based on command-line arguments
     int pipeDroneWriteSizeFD = atoi(argv[1]);
     int pipeDroneWriteKeyFD  = atoi(argv[2]);
     int pipeInputReadKeyFD   = atoi(argv[3]);
 
     // Global variables for writing obstacle and target data
     pipeDroneWriteObstaclesFD = atoi(argv[4]);
     pipeDroneWriteTargetsFD   = atoi(argv[5]);
 
     int pipeFD[2], pipe2FD[2], pipe3FD[2];
     if (pipe(pipeFD) == -1)
     {
         perror("[SERVER]: Error creating pipe for map size");
         LOG_TO_FILE(errorLog, "Error creating pipe for map size");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     if (pipe(pipe2FD) == -1)
     {
         perror("[SERVER]: Error creating second pipe for obstacles");
         LOG_TO_FILE(errorLog, "Error creating second pipe for obstacles");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     if (pipe(pipe3FD) == -1)
     {
         perror("[SERVER]: Error creating third pipe for targets");
         LOG_TO_FILE(errorLog, "Error creating third pipe for targets");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     int pipeMapReadSizeFD = pipeFD[0];
     pipeMapWriteObstaclesFD = pipe2FD[1];
     pipeMapWriteTargetsFD   = pipe3FD[1];
     char pipeDroneWriteSizeStr[10], pipeMapReadObstacleStr[10], pipeMapReadTargetStr[10];
     snprintf(pipeDroneWriteSizeStr, sizeof(pipeDroneWriteSizeStr), "%d", pipeFD[1]);
     snprintf(pipeMapReadObstacleStr, sizeof(pipeMapReadObstacleStr), "%d", pipe2FD[0]);
     snprintf(pipeMapReadTargetStr, sizeof(pipeMapReadTargetStr), "%d", pipe3FD[0]);
 
     // Create shared memory segments for drone and score
     int droneSharedMemFD = createDroneSharedMemory();
     int scoreSharedMemFD = createScoreSharedMemory();
 
     // Create and initialize a semaphore for drone synchronization
     sem_unlink("drone_sem");
     sharedDrone->sem = sem_open("drone_sem", O_CREAT | O_RDWR, 0666, 1);
     if (sharedDrone->sem == SEM_FAILED)
     {
         perror("[SERVER]: Error creating the drone semaphore");
         LOG_TO_FILE(errorLog, "Error creating the drone semaphore");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     // Create a semaphore to wait for the map window to start
     sem_unlink("/map_sem");
     sem_t* mapSemaphore = sem_open("/map_sem", O_CREAT | O_EXCL, 0666, 0);
     if (mapSemaphore == SEM_FAILED)
     {
         perror("[SERVER]: Failed to open the map semaphore");
         LOG_TO_FILE(errorLog, "Failed to open the map semaphore");
         exit(EXIT_FAILURE);
     }
 
     // Set initial configuration for the drone using command-line parameters
     sem_wait(sharedDrone->sem);
     LOG_TO_FILE(debugLog, "Initialized initial position for the drone");
     sscanf(argv[6], "%f,%f", &sharedDrone->pos_x, &sharedDrone->pos_y);
     sscanf(argv[7], "%f,%f", &sharedDrone->vel_x, &sharedDrone->vel_y);
     sscanf(argv[8], "%f,%f", &sharedDrone->force_x, &sharedDrone->force_y);
     sem_post(sharedDrone->sem);
     sem_close(sharedDrone->sem);
 
     // Launch the map window process using Konsole
     char* mapWindowCmd[] = {"konsole", "-e", "./map_window", pipeDroneWriteSizeStr, pipeMapReadObstacleStr, pipeMapReadTargetStr, NULL};
     pid_t konsoleMapPID = fork();
     if (konsoleMapPID < 0)
     {
         perror("[SERVER]: Error forking the map window process");
         LOG_TO_FILE(errorLog, "Error forking the map window process");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     else if (konsoleMapPID == 0)
     {
         execvp(mapWindowCmd[0], mapWindowCmd);
         perror("[SERVER]: Failed to execute the map window process");
         LOG_TO_FILE(errorLog, "Failed to execute the map window process");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     else
     {
         sem_wait(mapSemaphore);
         mapWindowPID = getTerminalChildPID(konsoleMapPID);
         sem_close(mapSemaphore);
     }
     
     // Set up signal handlers for SIGUSR1 and SIGUSR2
     struct sigaction sigAct;
     sigAct.sa_flags = SA_SIGINFO;
     sigAct.sa_sigaction = signalHandler;
     sigemptyset(&sigAct.sa_mask);
     if (sigaction(SIGUSR1, &sigAct, NULL) == -1)
     {
         perror("[SERVER]: Error in sigaction(SIGUSR1)");
         LOG_TO_FILE(errorLog, "Error in sigaction(SIGUSR1)");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     if (sigaction(SIGUSR2, &sigAct, NULL) == -1)
     {
         perror("[SERVER]: Error in sigaction(SIGUSR2)");
         LOG_TO_FILE(errorLog, "Error in sigaction(SIGUSR2)");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     sigset_t signalSet;
     sigfillset(&signalSet);
     sigdelset(&signalSet, SIGUSR1);
     sigdelset(&signalSet, SIGUSR2);
     sigprocmask(SIG_SETMASK, &signalSet, NULL);
     
     // Initialize and run the subscriber application
     if (serverApp->init())
     {
         serverApp->run(pipeDroneWriteSizeFD, pipeDroneWriteKeyFD, pipeInputReadKeyFD, pipeMapReadSizeFD);
     }
 
     // END PROGRAM: Unlink and clean up shared resources
     if (shm_unlink(DRONE_SHARED_MEMORY) == -1 || shm_unlink(SCORE_SHARED_MEMORY) == -1)
     {
         perror("[SERVER]: Error unlinking shared memory");
         LOG_TO_FILE(errorLog, "Error unlinking the drone or score shared memory");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     if (close(droneSharedMemFD) == -1 || close(scoreSharedMemFD) == -1)
     {
         perror("[SERVER]: Error closing shared memory file descriptors");
         LOG_TO_FILE(errorLog, "Error closing shared memory file descriptors");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     munmap(sharedDrone, sizeof(Drone));
     munmap(sharedScore, sizeof(float));
 
     // Unlink semaphores and close log files
     sem_unlink("drone_sem");
     sem_unlink("/map_sem");
     fclose(debugLog);
     fclose(errorLog);
 
     delete serverApp;
     return 0;
 }
 