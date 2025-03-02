/**
 * TargetPub.cpp
 * 
 * This program implements a Fast DDS publisher for "TargetsTopic". It publishes messages containing target data,
 * which include sequences of x and y coordinates and the total number of targets.
 * The publisher uses a custom DataWriterListener to track publication matching.
 */

 #include <chrono>
 #include <thread>
 #include <iostream>
 #include <random>
 #include <atomic>
 #include <cstring>
 #include <cstdlib>
 
 #include <fastdds/dds/domain/DomainParticipant.hpp>
 #include <fastdds/dds/domain/DomainParticipantFactory.hpp>
 #include <fastdds/dds/publisher/DataWriter.hpp>
 #include <fastdds/dds/publisher/DataWriterListener.hpp>
 #include <fastdds/dds/publisher/Publisher.hpp>
 #include <fastdds/dds/topic/TypeSupport.hpp>
 
 #include "utils.h"
 #include "Generated/ObjectsPubSubTypes.hpp"
 #include <signal.h>
 #include <semaphore.h>
 
 // Use Fast DDS DDS namespace
 using namespace eprosima::fastdds::dds;
 
 // Global variables for logging, process IDs, and synchronization
 FILE* debugLog, *errorLog;
 pid_t watchdogPID;
 sem_t* syncSemaphore = nullptr;
 bool globalMatched = false;
 
 //------------------------------------------------------------------------------
 // Class: TargetPublisher
 //------------------------------------------------------------------------------
 class TargetPublisher
 {
 private:
     // Message to be published (contains target data)
     Objects targetMessage_;
 
     // Fast DDS entities
     DomainParticipant* participant_;
     Publisher* publisher_;
     Topic* topic_;
     DataWriter* writer_;
     TypeSupport typeSupport_;
 
     // Custom DataWriterListener class to handle publication matching events
     class PubListener : public DataWriterListener
     {
     public:
         PubListener() : matchCount_(0) {}
         ~PubListener() override {}
 
         // Called when a publication is matched or unmatched
         void on_publication_matched(DataWriter* /*writer*/, const PublicationMatchedStatus& info) override
         {
             if (info.current_count_change == 1)
             {
                 matchCount_ = info.total_count;
                 std::cout << "Target publisher matched." << std::endl;
                 // Signal synchronization and wait a short period before starting publishing
                 sem_post(syncSemaphore);
                 std::this_thread::sleep_for(std::chrono::seconds(3));
                 globalMatched = true;
             }
             else if (info.current_count_change == -1)
             {
                 matchCount_ = info.total_count;
                 globalMatched = false;
                 std::cout << "Target publisher unmatched." << std::endl;
             }
             else
             {
                 std::cout << info.current_count_change
                           << " is not a valid change value for publication matching." << std::endl;
             }
         }
 
         std::atomic_int matchCount_;
     } pubListener_;
 
 public:
     // Configuration parameters
     int numTargets;      // Number of targets to generate
     int mapWidth;        // Map width dimension
     int mapHeight;       // Map height dimension
 
     // Constructor
     TargetPublisher()
         : participant_(nullptr)
         , publisher_(nullptr)
         , topic_(nullptr)
         , writer_(nullptr)
         , typeSupport_(new ObjectsPubSubType())
         , numTargets(0)
         , mapWidth(0)
         , mapHeight(0)
     {}
 
     // Destructor: clean up Fast DDS entities
     virtual ~TargetPublisher()
     {
         if (writer_ != nullptr)
         {
             publisher_->delete_datawriter(writer_);
         }
         if (publisher_ != nullptr)
         {
             participant_->delete_publisher(publisher_);
         }
         if (topic_ != nullptr)
         {
             participant_->delete_topic(topic_);
         }
         DomainParticipantFactory::get_instance()->delete_participant(participant_);
     }
 
     //! Initialize the publisher by creating the participant, registering the type, creating the topic, publisher, and datawriter.
     bool init()
     {
         DomainParticipantQos participantQos;
         participantQos.name("TargetPublisherParticipant");
         
         // Create the DomainParticipant with domain ID 2
         participant_ = DomainParticipantFactory::get_instance()->create_participant(2, participantQos);
         if (participant_ == nullptr)
         {
             return false;
         }
 
         // Register the type support for Objects
         typeSupport_.register_type(participant_);
 
         // Create the Topic "TargetsTopic"
         topic_ = participant_->create_topic("TargetsTopic", typeSupport_.get_type_name(), TOPIC_QOS_DEFAULT);
         if (topic_ == nullptr)
         {
             return false;
         }
 
         // Create the Publisher
         publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
         if (publisher_ == nullptr)
         {
             return false;
         }
 
         // Create the DataWriter with the custom listener
         writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &pubListener_);
         if (writer_ == nullptr)
         {
             return false;
         }
 
         return true;
     }
 
     //! Generate target data and populate the message
     void generateTargets(int targetCount, int mapW, int mapH)
     {
         // Random number generation for target positions
         std::random_device rd;                              // Random seed
         std::mt19937 generator(rd());                       // Mersenne Twister RNG
         std::uniform_int_distribution<> distributionX(1, mapW - 2); // Range for x positions
         std::uniform_int_distribution<> distributionY(1, mapH - 2); // Range for y positions
 
         // Clear any previous data in the message
         targetMessage_.x().clear();
         targetMessage_.y().clear();
         targetMessage_.objects_number() = targetCount;
 
         // Generate random target positions
         for (int i = 0; i < targetCount; i++)
         {
             targetMessage_.x().push_back(distributionX(generator));
             targetMessage_.y().push_back(distributionY(generator));
         }
     }
 
     //! Publish the target data message
     bool publish()
     {
         if (pubListener_.matchCount_ > 0)
         {
             generateTargets(numTargets, mapWidth, mapHeight);
             writer_->write(&targetMessage_);
             return true;
         }
         return false;
     }
 
     //! Run the publishing loop
     void run()
     {
         while (true)
         {
             if (globalMatched)
             {
                 publish();
                 // Wait 15 seconds between publications
                 std::this_thread::sleep_for(std::chrono::seconds(15));
             }
         }
     }
 };
 
 // Global pointer to the TargetPublisher instance
 TargetPublisher* targetPublisher = nullptr;
 
 //------------------------------------------------------------------------------
 // Signal handler for graceful shutdown
 //------------------------------------------------------------------------------
 void signal_handler(int sig, siginfo_t* info, void* context)
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
         std::cout << "Target publisher shutting down: " << getpid() << std::endl;
         delete targetPublisher;
         fclose(errorLog);
         fclose(debugLog);
         exit(EXIT_SUCCESS);
     }
 }
 
 //------------------------------------------------------------------------------
 // Main function: initializes and runs the target publisher
 //------------------------------------------------------------------------------
 int main(int argc, char* argv[])
 {
     std::cout << "Starting Target publisher." << std::endl;
 
     // Create the TargetPublisher instance
     targetPublisher = new TargetPublisher();
 
     // Open log files for debugging and error logging
     debugLog = fopen("debug.log", "a");
     if (debugLog == NULL)
     {
         perror("[TARGET]: Error opening the debug log file");
         exit(EXIT_FAILURE);
     }
     errorLog = fopen("errors.log", "a");
     if (errorLog == NULL)
     {
         perror("[TARGET]: Error opening the error log file");
         exit(EXIT_FAILURE);
     }
 
     // Check for the required number of command-line arguments
     if (argc < 4)
     {
         LOG_TO_FILE(errorLog, "Invalid number of parameters");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
 
     LOG_TO_FILE(debugLog, "Process started");
 
     // Open the synchronization semaphore for publisher coordination
     sem_t* execSemaphore = sem_open("/exec_semaphore", 0);
     if (execSemaphore == SEM_FAILED)
     {
         perror("[TARGET]: Failed to open the exec semaphore");
         LOG_TO_FILE(errorLog, "Failed to open the exec semaphore");
         exit(EXIT_FAILURE);
     }
     sem_post(execSemaphore);
     sem_close(execSemaphore);
 
     // Open the semaphore to synchronize publishers
     syncSemaphore = sem_open("/sync_semaphore", 0);
     if (syncSemaphore == SEM_FAILED)
     {
         perror("[TARGET]: Failed to open the sync semaphore");
         LOG_TO_FILE(errorLog, "Failed to open the sync semaphore");
         exit(EXIT_FAILURE);
     }
 
     // Set configuration parameters from command-line arguments
     targetPublisher->numTargets = atoi(argv[1]);
     targetPublisher->mapWidth   = atoi(argv[2]);
     targetPublisher->mapHeight  = atoi(argv[3]);
 
     // Set up signal handling for graceful shutdown and watchdog communication
     struct sigaction sigAct;
     sigAct.sa_flags = SA_SIGINFO;
     sigAct.sa_sigaction = signal_handler;
     sigemptyset(&sigAct.sa_mask);
     if (sigaction(SIGUSR1, &sigAct, NULL) == -1)
     {
         perror("[TARGET]: Error in sigaction(SIGUSR1)");
         LOG_TO_FILE(errorLog, "Error in sigaction(SIGUSR1)");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     if (sigaction(SIGUSR2, &sigAct, NULL) == -1)
     {
         perror("[TARGET]: Error in sigaction(SIGUSR2)");
         LOG_TO_FILE(errorLog, "Error in sigaction(SIGUSR2)");
         fclose(debugLog);
         fclose(errorLog);
         exit(EXIT_FAILURE);
     }
     sigset_t sigSet;
     sigfillset(&sigSet);
     sigdelset(&sigSet, SIGUSR1);
     sigdelset(&sigSet, SIGUSR2);
     sigdelset(&sigSet, SIGTERM);
     sigprocmask(SIG_SETMASK, &sigSet, NULL);
 
     // Initialize the publisher and start the publishing loop
     if (targetPublisher->init())
     {
         targetPublisher->run();
     }
 
     // Cleanup before exit (unreachable in current infinite loop design)
     sem_close(syncSemaphore);
     delete targetPublisher;
     fclose(debugLog);
     fclose(errorLog);
     return 0;
 }
 