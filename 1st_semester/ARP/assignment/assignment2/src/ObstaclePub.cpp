// ObstaclePub.cpp
// This file implements a publisher for obstacle objects using Fast DDS.
// It creates a DomainParticipant, registers the type, creates a topic, publisher, 
// and data writer, then continuously publishes randomly generated obstacle data.

#include <chrono>
#include <thread>
#include <random>
#include <iostream>
#include <signal.h>
#include <semaphore.h>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "utils.h"
#include "Generated/ObjectsPubSubTypes.hpp"

using namespace eprosima::fastdds::dds;

// Global variables for logging, watchdog PID, and synchronization
FILE* debugLog = nullptr;
FILE* errorLog = nullptr;
pid_t watchdogPID;
bool globalMatched = false;
sem_t* syncSemaphore = nullptr;

// ObstaclePub class encapsulates the obstacle publisher functionality
class ObstaclePublisher {
private:
    Objects obstacleMessage;             // Message object to be published
    DomainParticipant* participant;      // Fast DDS DomainParticipant
    Publisher* publisher;                // Fast DDS Publisher
    Topic* topic;                        // Topic for obstacles
    DataWriter* dataWriter;              // Data writer for publishing
    TypeSupport typeSupport;             // Type support for the message type

    // Inner class: Custom DataWriterListener to monitor publication matching
    class PublisherListener : public DataWriterListener {
    public:
        PublisherListener() : matchedCount_(0) {}
        ~PublisherListener() override {}

        // Callback when a publication is matched/unmatched
        void on_publication_matched(
            DataWriter* /*writer*/,
            const PublicationMatchedStatus& status) override
        {
            if (status.current_count_change == 1)
            {
                matchedCount_ = status.total_count;
                std::cout << "Obstacle Publisher matched." << std::endl;
                // Wait until the other publisher is ready to publish
                sem_wait(syncSemaphore);
                // Sleep for 3 seconds before starting publication
                std::this_thread::sleep_for(std::chrono::seconds(3));
                globalMatched = true;
            }
            else if (status.current_count_change == -1)
            {
                matchedCount_ = status.total_count;
                globalMatched = false;
                std::cout << "Obstacle Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << status.current_count_change
                          << " is not a valid value for publication matching change." << std::endl;
            }
        }

        std::atomic_int matchedCount_;
    } listener;

public:
    // Configuration parameters for obstacles and map dimensions
    int numObstacles;
    int mapWidth, mapHeight;

    // Constructor: Initializes pointers to nullptr and creates a new type support object
    ObstaclePublisher()
        : participant(nullptr)
        , publisher(nullptr)
        , topic(nullptr)
        , dataWriter(nullptr)
        , typeSupport(new ObjectsPubSubType())
    {}

    // Destructor: Cleans up Fast DDS entities and deletes the participant
    virtual ~ObstaclePublisher() {
        if (dataWriter != nullptr) {
            publisher->delete_datawriter(dataWriter);
        }
        if (publisher != nullptr) {
            participant->delete_publisher(publisher);
        }
        if (topic != nullptr) {
            participant->delete_topic(topic);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant);
    }

    //! Initialize the obstacle publisher.
    bool init() {
        // Set participant QoS and create a DomainParticipant on domain 2.
        DomainParticipantQos participantQos;
        participantQos.name("ObstaclePublisherParticipant");
        participant = DomainParticipantFactory::get_instance()->create_participant(2, participantQos);
        if (participant == nullptr) {
            return false;
        }

        // Register the message type with the DomainParticipant.
        typeSupport.register_type(participant);

        // Create the topic for obstacles.
        topic = participant->create_topic("ObstaclesTopic", typeSupport.get_type_name(), TOPIC_QOS_DEFAULT);
        if (topic == nullptr) {
            return false;
        }

        // Create a Publisher.
        publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
        if (publisher == nullptr) {
            return false;
        }

        // Create a DataWriter for the topic with the custom listener.
        dataWriter = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, &listener);
        if (dataWriter == nullptr) {
            return false;
        }

        return true;
    }

    //! Publish a new obstacle message if the publisher is matched.
    bool publish() {
        if (listener.matchedCount_ > 0) {
            generate_obstacle_data(numObstacles, mapWidth, mapHeight);
            dataWriter->write(&obstacleMessage);
            return true;
        }
        return false;
    }

    //! Run the publisher loop that periodically publishes obstacle data.
    void run() {
        while (true) {
            if (globalMatched) {
                publish();
                std::this_thread::sleep_for(std::chrono::seconds(15));
            }
        }
    }

    //! Generate random obstacle positions and update the message.
    void generate_obstacle_data(int obstacleCount, int width, int height) {
        std::random_device rd;                              // Seed for randomness
        std::mt19937 generator(rd());                      // Mersenne Twister generator
        std::uniform_int_distribution<> distX(1, width - 2); // Uniform distribution for x positions
        std::uniform_int_distribution<> distY(1, height - 2); // Uniform distribution for y positions

        // Clear existing data
        obstacleMessage.x().clear();
        obstacleMessage.y().clear();
        obstacleMessage.objects_number() = obstacleCount;

        // Generate random positions for each obstacle.
        for (int i = 0; i < obstacleCount; i++) {
            obstacleMessage.x().push_back(distX(generator));
            obstacleMessage.y().push_back(distY(generator));
        }
    }
};

ObstaclePublisher* obstaclePublisher;  // Global pointer to the ObstaclePublisher instance

//! Signal handler for graceful shutdown triggered by the watchdog.
void signal_handler(int sig, siginfo_t* info, void* context) {
    (void) context; // Unused parameter
    if (sig == SIGUSR1) {
        watchdogPID = info->si_pid;
        LOG_TO_FILE(debugLog, "Signal SIGUSR1 received from WATCHDOG");
        kill(watchdogPID, SIGUSR1);
    }
    if (sig == SIGUSR2) {
        LOG_TO_FILE(debugLog, "Shutting down by the WATCHDOG");
        std::cout << "Obstacle Publisher shutting down by the WATCHDOG: " << getpid() << std::endl;
        delete obstaclePublisher;
        fclose(errorLog);
        fclose(debugLog);
        exit(EXIT_SUCCESS);
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Starting Obstacle Publisher" << std::endl;

    // Create an instance of ObstaclePublisher.
    obstaclePublisher = new ObstaclePublisher();

    // Open log files for debugging and errors.
    debugLog = fopen("debug.log", "a");
    if (debugLog == nullptr) {
        perror("[OBSTACLE]: Error opening the debug log file");
        exit(EXIT_FAILURE);
    }
    errorLog = fopen("errors.log", "a");
    if (errorLog == nullptr) {
        perror("[OBSTACLE]: Error opening the error log file");
        exit(EXIT_FAILURE);
    }

    // Check command line arguments: expecting at least 4 parameters (program name, number of obstacles, map width, map height).
    if (argc < 4) {
        LOG_TO_FILE(errorLog, "Invalid number of parameters");
        fclose(debugLog);
        fclose(errorLog);
        exit(EXIT_FAILURE);
    }

    LOG_TO_FILE(debugLog, "Process started");

    // Open and post the execution semaphore for inter-process synchronization.
    sem_t* execSemaphore = sem_open("/exec_semaphore", 0);
    if (execSemaphore == SEM_FAILED) {
        perror("[OBSTACLE]: Failed to open the execution semaphore");
        LOG_TO_FILE(errorLog, "Failed to open the execution semaphore");
        exit(EXIT_FAILURE);
    }
    sem_post(execSemaphore);  // Release semaphore to allow other processes to start.
    sem_close(execSemaphore);

    // Open the synchronization semaphore between publishers.
    syncSemaphore = sem_open("/sync_semaphore", 0);
    if (syncSemaphore == SEM_FAILED) {
        perror("[OBSTACLE]: Failed to open the sync semaphore");
        LOG_TO_FILE(errorLog, "Failed to open the sync semaphore");
        exit(EXIT_FAILURE);
    }

    // Parse command line arguments for configuration.
    obstaclePublisher->numObstacles = atoi(argv[1]);
    obstaclePublisher->mapWidth = atoi(argv[2]);
    obstaclePublisher->mapHeight = atoi(argv[3]);

    // Set up signal handlers for SIGUSR1 and SIGUSR2.
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = signal_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        perror("[OBSTACLE]: Error setting signal handler for SIGUSR1");
        LOG_TO_FILE(errorLog, "Error setting signal handler for SIGUSR1");
        fclose(debugLog);
        fclose(errorLog);
        exit(EXIT_FAILURE);
    }
    if (sigaction(SIGUSR2, &sa, NULL) == -1) {
        perror("[OBSTACLE]: Error setting signal handler for SIGUSR2");
        LOG_TO_FILE(errorLog, "Error setting signal handler for SIGUSR2");
        fclose(debugLog);
        fclose(errorLog);
        exit(EXIT_FAILURE);
    }

    // Block all signals except SIGUSR1, SIGUSR2, and SIGTERM.
    sigset_t signalSet;
    sigfillset(&signalSet);
    sigdelset(&signalSet, SIGUSR1);
    sigdelset(&signalSet, SIGUSR2);
    sigdelset(&signalSet, SIGTERM);
    sigprocmask(SIG_SETMASK, &signalSet, NULL);

    // Initialize the obstacle publisher.
    if (obstaclePublisher->init()) {
        // Start the publisher loop.
        obstaclePublisher->run();
    }

    // Clean up before exit.
    sem_close(syncSemaphore);
    delete obstaclePublisher;
    fclose(debugLog);
    fclose(errorLog);
    return 0;
}
