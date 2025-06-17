
#ifndef __CARAPP_H_
#define __CARAPP_H_

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>

#include "inet/applications/base/ApplicationBase.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TimeTag_m.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"

#include "RDOAgent.h"  // Include RDOAgent

/**
 * CarApp - Application for NRCars in 5G V2X communication
 * Handles mobility updates, packet transmission, RDO optimization, and DoS attack simulation
 */
class CarApp : public inet::ApplicationBase
{
  private:
    // Attack types - only DoS supported now
    enum {
        ATTACK_TYPE_NONE = 0,
        ATTACK_TYPE_DOS = 1
        // Removed ATTACK_TYPE_SYBIL and ATTACK_TYPE_FALSIFIED
    };

    // Interface table and UDP socket
    inet::IInterfaceTable *interfaceTable = nullptr;
    inet::UdpSocket socket;

    // Timers
    omnetpp::cMessage *mobilityUpdateMsg = nullptr;
    omnetpp::cMessage *sendMessageMsg = nullptr;
    omnetpp::cMessage *attackMsg = nullptr;
    omnetpp::cMessage *rdoUpdateEvent = nullptr;

    // Attack events
    struct AttackEvent {
        double time;
        int attackType;
        bool operator<(const AttackEvent& other) const {
            return time < other.time;
        }
    };

    // Vehicle information
    int mySenderId = 0;
    unsigned int seqNumber = 0;
    std::string mobilityFile;
    std::vector<int> fakeIds;  // Kept for compatibility but not used

    // Current mobility state
    double currentX = 0.0;
    double currentY = 0.0;
    double currentZ = 0.0;
    double currentSpeed = 0.0;
    double currentDirection = 0.0;

    double costFunctionTarget;  // The target value for this node's cost function (ai)
    double costFunctionWeight;  // Weight parameter for the cost function

    // Metrics for tracking RDO communication
    double lastReceivedTime;               // Time of last received RDO packet
    int totalRdoPacketsReceived;           // Counter for received RDO packets
    int totalRdoPacketsSent;               // Counter for sent RDO packets

    // Attack state
    bool isAttacking = false;
    int currentAttackType = ATTACK_TYPE_NONE;

    // Mobility and Attack Data
    std::vector<std::pair<double, std::pair<double, double>>> mobilityData;
    size_t mobilityDataIndex = 0;
    std::vector<AttackEvent> attackSchedule;

    // Statistics vectors
    omnetpp::cOutVector packetSentVector;
    omnetpp::cOutVector attackRateVector;
    omnetpp::cOutVector positionXVector;
    omnetpp::cOutVector positionYVector;
    omnetpp::cOutVector speedVector;

    // RDO-related fields
    bool rdoEnabled = false;
    bool forceTrusted = false;
    RDOAgent *rdoAgent = nullptr;
    std::map<int, double> neighborStates;  // Neighbor RDO states
    std::vector<int> trustedGnbIds = {0, 1, 2};  // Trusted gNBs
    bool isTrusted = false;
    int loadedFilterType = 1;
    int loadedConsensusAlgorithm = 1;

    omnetpp::cOutVector rdoStateVector;
    omnetpp::cOutVector rdoGradientVector;
    omnetpp::cOutVector rdoConsensusVector;  // Added for consensus tracking

    // Simulation signals
    omnetpp::simsignal_t sinrSignal;
    omnetpp::simsignal_t packetSentSignal;
    omnetpp::simsignal_t packetReceivedSignal;
    omnetpp::simsignal_t endToEndDelaySignal;

  protected:
    // Core lifecycle methods
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(omnetpp::cMessage *msg) override;
    virtual void finish() override;
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;

    // Utility methods
    void loadMobilityData();
    void updateMobility();
    void loadAttackSchedule(const std::string& filename);
    void checkForAttack();
    void startAttack(int attackType);
    void stopAttack();
    int getWlanInterfaceId();
    void sendToNic(inet::Packet *packet);

    // Message handling
    void sendNormalMessage();
    void sendDoSAttack();
    // Removed sendSybilAttack() and sendFalsifiedData() methods
    void processReceivedPacket(inet::Packet *packet);

    // RDO methods
    void initializeRDO();
    void processRDOUpdate();
    void sendRDOState();
    void processRDOStatePacket(inet::Packet *packet);
    void loadRDOConfig(const std::string& filename);
    double generateGradient();

  public:
    CarApp() = default;
    virtual ~CarApp();
};

#endif // __CARAPP_H_

///////Update working code////

//
//#ifndef __CARAPP_H_
//#define __CARAPP_H_
//
//#include <string>
//#include <vector>
//#include <map>
//#include <algorithm>
//#include <fstream>
//#include <sstream>
//#include <cmath>
//
//#include "inet/applications/base/ApplicationBase.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/common/TimeTag_m.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/networklayer/common/InterfaceTable.h"
//#include "inet/transportlayer/contract/udp/UdpSocket.h"
//
//#include "RDOAgent.h"  // Include RDOAgent
//
///**
// * CarApp - Application for NRCars in 5G V2X communication
// * Handles mobility updates, packet transmission, RDO optimization, and DoS attack simulation
// */
//class CarApp : public inet::ApplicationBase
//{
//  private:
//    // Attack types - only DoS supported now
//    enum {
//        ATTACK_TYPE_NONE = 0,
//        ATTACK_TYPE_DOS = 1
//        // Removed ATTACK_TYPE_SYBIL and ATTACK_TYPE_FALSIFIED
//    };
//
//    // Interface table and UDP socket
//    inet::IInterfaceTable *interfaceTable = nullptr;
//    inet::UdpSocket socket;
//
//    // Timers
//    omnetpp::cMessage *mobilityUpdateMsg = nullptr;
//    omnetpp::cMessage *sendMessageMsg = nullptr;
//    omnetpp::cMessage *attackMsg = nullptr;
//    omnetpp::cMessage *rdoUpdateEvent = nullptr;
//
//    // Attack events
//    struct AttackEvent {
//        double time;
//        int attackType;
//        bool operator<(const AttackEvent& other) const {
//            return time < other.time;
//        }
//    };
//
//    // Vehicle information
//    int mySenderId = 0;
//    unsigned int seqNumber = 0;
//    std::string mobilityFile;
//    std::vector<int> fakeIds;  // Kept for compatibility but not used
//
//    // Current mobility state
//    double currentX = 0.0;
//    double currentY = 0.0;
//    double currentZ = 0.0;
//    double currentSpeed = 0.0;
//    double currentDirection = 0.0;
//
//    double costFunctionTarget;  // The target value for this node's cost function (ai)
//    double costFunctionWeight;  // Weight parameter for the cost function
//
//    // Metrics for tracking RDO communication
//    double lastReceivedTime;               // Time of last received RDO packet
//    int totalRdoPacketsReceived;           // Counter for received RDO packets
//    int totalRdoPacketsSent;               // Counter for sent RDO packets
//
//    // Attack state
//    bool isAttacking = false;
//    int currentAttackType = ATTACK_TYPE_NONE;
//
//    // Mobility and Attack Data
//    std::vector<std::pair<double, std::pair<double, double>>> mobilityData;
//    size_t mobilityDataIndex = 0;
//    std::vector<AttackEvent> attackSchedule;
//
//    // Statistics vectors
//    omnetpp::cOutVector packetSentVector;
//    omnetpp::cOutVector attackRateVector;
//    omnetpp::cOutVector positionXVector;
//    omnetpp::cOutVector positionYVector;
//    omnetpp::cOutVector speedVector;
//
//    // RDO-related fields
//    bool rdoEnabled = false;
//    RDOAgent *rdoAgent = nullptr;
//    std::map<int, double> neighborStates;  // Neighbor RDO states
//    std::vector<int> trustedGnbIds = {0, 1, 2};  // Trusted gNBs
//    bool isTrusted = false;
//    int loadedFilterType = 1;
//    int loadedConsensusAlgorithm = 1;
//
//    omnetpp::cOutVector rdoStateVector;
//    omnetpp::cOutVector rdoGradientVector;
//    omnetpp::cOutVector rdoConsensusVector;  // Added for consensus tracking
//
//    // Simulation signals
//    omnetpp::simsignal_t sinrSignal;
//    omnetpp::simsignal_t packetSentSignal;
//    omnetpp::simsignal_t packetReceivedSignal;
//    omnetpp::simsignal_t endToEndDelaySignal;
//
//  protected:
//    // Core lifecycle methods
//    virtual void initialize(int stage) override;
//    virtual void handleMessageWhenUp(omnetpp::cMessage *msg) override;
//    virtual void finish() override;
//    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
//    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;
//
//    // Utility methods
//    void loadMobilityData();
//    void updateMobility();
//    void loadAttackSchedule(const std::string& filename);
//    void checkForAttack();
//    void startAttack(int attackType);
//    void stopAttack();
//    int getWlanInterfaceId();
//    void sendToNic(inet::Packet *packet);
//
//    // Message handling
//    void sendNormalMessage();
//    void sendDoSAttack();
//    // Removed sendSybilAttack() and sendFalsifiedData() methods
//    void processReceivedPacket(inet::Packet *packet);
//
//    // RDO methods
//    void initializeRDO();
//    void processRDOUpdate();
//    void sendRDOState();
//    void processRDOStatePacket(inet::Packet *packet);
//    void loadRDOConfig(const std::string& filename);
//    double generateGradient();
//
//  public:
//    CarApp() = default;
//    virtual ~CarApp();
//};
//
//#endif // __CARAPP_H_


/////Working code with RDO-T//////

//#ifndef __CARAPP_H_
//#define __CARAPP_H_
//
//#include <string>
//#include <vector>
//#include <map>
//#include <algorithm>
//#include <fstream>
//#include <sstream>
//#include <cmath>
//
//#include "inet/applications/base/ApplicationBase.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/common/TimeTag_m.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/networklayer/common/InterfaceTable.h"
//#include "inet/transportlayer/contract/udp/UdpSocket.h"
//
//#include "RDOAgent.h"  // Include RDOAgent
//
///**
// * CarApp - Application for NRCars in 5G V2X communication
// * Handles mobility updates, packet transmission, RDO optimization, and attack simulation
// */
//class CarApp : public inet::ApplicationBase
//{
//  private:
//    // Attack types
//    enum {
//        ATTACK_TYPE_NONE = 0,
//        ATTACK_TYPE_DOS = 1,
//        ATTACK_TYPE_SYBIL = 2,
//        ATTACK_TYPE_FALSIFIED = 3
//    };
//
//    // Interface table and UDP socket
//    inet::IInterfaceTable *interfaceTable = nullptr;
//    inet::UdpSocket socket;
//
//    // Timers
//    omnetpp::cMessage *mobilityUpdateMsg = nullptr;
//    omnetpp::cMessage *sendMessageMsg = nullptr;
//    omnetpp::cMessage *attackMsg = nullptr;
//    omnetpp::cMessage *rdoUpdateEvent = nullptr;
//
//    // Attack events
//    struct AttackEvent {
//        double time;
//        int attackType;
//        bool operator<(const AttackEvent& other) const {
//            return time < other.time;
//        }
//    };
//
//    // Vehicle information
//    int mySenderId = 0;
//    unsigned int seqNumber = 0;
//    std::string mobilityFile;
//    std::vector<int> fakeIds;  // For Sybil attack
//
//    // Current mobility state
//    double currentX = 0.0;
//    double currentY = 0.0;
//    double currentZ = 0.0;
//    double currentSpeed = 0.0;
//    double currentDirection = 0.0;
//
//    // Attack state
//    bool isAttacking = false;
//    int currentAttackType = ATTACK_TYPE_NONE;
//
//    // Mobility and Attack Data
//    std::vector<std::pair<double, std::pair<double, double>>> mobilityData;
//    size_t mobilityDataIndex = 0;
//    std::vector<AttackEvent> attackSchedule;
//
//    // Statistics vectors
//    omnetpp::cOutVector packetSentVector;
//    omnetpp::cOutVector attackRateVector;
//    omnetpp::cOutVector positionXVector;
//    omnetpp::cOutVector positionYVector;
//    omnetpp::cOutVector speedVector;
//
//    // ✅ RDO-related fields
//    bool rdoEnabled = false;
//    RDOAgent *rdoAgent = nullptr;
//    std::map<int, double> neighborStates;  // Neighbor RDO states
//    std::vector<int> trustedGnbIds = {0, 1, 2};  // Trusted gNBs
//    bool isTrusted = false;
//    int loadedFilterType = 1;
//    int loadedConsensusAlgorithm = 1;
//
//    omnetpp::cOutVector rdoStateVector;
//    omnetpp::cOutVector rdoGradientVector;
//
//    // ✅ Simulation signals
//    omnetpp::simsignal_t sinrSignal;
//    omnetpp::simsignal_t packetSentSignal;
//    omnetpp::simsignal_t packetReceivedSignal;
//    omnetpp::simsignal_t endToEndDelaySignal;
//
//  protected:
//    // Core lifecycle methods
//    virtual void initialize(int stage) override;
//    virtual void handleMessageWhenUp(omnetpp::cMessage *msg) override;
//    virtual void finish() override;
//    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
//    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;
//
//    // Utility methods
//    void loadMobilityData();
//    void updateMobility();
//    void loadAttackSchedule(const std::string& filename);
//    void checkForAttack();
//    void startAttack(int attackType);
//    void stopAttack();
//    int getWlanInterfaceId();
//    void sendToNic(inet::Packet *packet);
//
//    // Message handling
//    void sendNormalMessage();
//    void sendDoSAttack();
//    void sendSybilAttack();
//    void sendFalsifiedData();
//    void processReceivedPacket(inet::Packet *packet);
//
//    // ✅ RDO methods
//    void initializeRDO();
//    void processRDOUpdate();
//    void sendRDOState();
//    void processRDOStatePacket(inet::Packet *packet);
//    void loadRDOConfig(const std::string& filename);
//    double generateGradient();
//
//
//  public:
//    CarApp() = default;
//    virtual ~CarApp();
//};
//
//#endif // __CARAPP_H_




////////WORKING PHASE1&2/////////


//#ifndef __CARAPP_H_
//#define __CARAPP_H_
//
//#include <string>
//#include <vector>
//#include <map>
//#include <algorithm>
//#include <fstream>
//#include <sstream>
//#include <cmath>
//
//#include "inet/applications/base/ApplicationBase.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/common/TimeTag_m.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/networklayer/common/InterfaceTable.h"
//
///**
// * CarApp - Application for NRCars in 5G V2X communication
// * Handles mobility updates, packet transmission, and attack simulation
// */
//class CarApp : public inet::ApplicationBase
//{
//  private:
//    // Attack types
//    enum {
//        ATTACK_TYPE_NONE = 0,
//        ATTACK_TYPE_DOS = 1,
//        ATTACK_TYPE_SYBIL = 2,
//        ATTACK_TYPE_FALSIFIED = 3
//    };
//
//    // Interface table module
//    inet::IInterfaceTable *interfaceTable = nullptr;
//
//    // Timers
//    omnetpp::cMessage *mobilityUpdateMsg = nullptr;
//    omnetpp::cMessage *sendMessageMsg = nullptr;
//    omnetpp::cMessage *attackMsg = nullptr;
//
//    // Attack events
//    struct AttackEvent {
//        double time;
//        int attackType;
//        bool operator<(const AttackEvent& other) const {
//            return time < other.time;
//        }
//    };
//
//    // Vehicle information
//    int mySenderId = 0;  // Derived from car[] index
//    unsigned int seqNumber = 0;
//    std::string mobilityFile;
//    std::vector<int> fakeIds;  // For Sybil attack
//
//    // Current state
//    double currentX = 0.0;
//    double currentY = 0.0;
//    double currentZ = 0.0;
//    double currentSpeed = 0.0;
//    double currentDirection = 0.0;
//
//    // Attack state
//    bool isAttacking = false;
//    int currentAttackType = ATTACK_TYPE_NONE;
//
//    // Mobility data storage
//    std::vector<std::pair<double, std::pair<double, double>>> mobilityData;
//    size_t mobilityDataIndex = 0;
//
//    // Attack schedule
//    std::vector<AttackEvent> attackSchedule;
//
//    // Statistics
//    omnetpp::cOutVector packetSentVector;
//    omnetpp::cOutVector attackRateVector;
//    omnetpp::cOutVector positionXVector;
//    omnetpp::cOutVector positionYVector;
//    omnetpp::cOutVector speedVector;
//
//  protected:
//    // Lifecycle methods
//    virtual void initialize(int stage) override;
//    virtual void handleMessageWhenUp(omnetpp::cMessage *msg) override;
//    virtual void finish() override;
//    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
//    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
//    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;
//
//    // Utility methods
//    void loadMobilityData();
//    void updateMobility();
//    void loadAttackSchedule(const std::string& filename);
//    void checkForAttack();
//    void startAttack(int attackType);
//    void stopAttack();
//    int getWlanInterfaceId();
//    void sendToNic(inet::Packet *packet);
//
//    // Message handling
//    void sendNormalMessage();
//    void sendDoSAttack();
//    void sendSybilAttack();
//    void sendFalsifiedData();
//    void processReceivedPacket(inet::Packet *packet);
//
//  public:
//    CarApp() = default;
//    virtual ~CarApp();
//};
//
//#endif // __CARAPP_H_



//#ifndef __CARAPP_H_
//#define __CARAPP_H_
//
//#include <omnetpp.h>
//#include <string>
//#include <fstream>
//#include <vector>
//#include <map>
//#include <algorithm>
//#include "inet/common/INETDefs.h"
//#include "inet/common/TimeTag_m.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/applications/base/ApplicationBase.h"
//#include "inet/networklayer/common/InterfaceTable.h"  // <-- ⭐ You also need this!
//
//using namespace omnetpp;
//using namespace inet;
//
//// Define attack types
//enum {
//    ATTACK_TYPE_NONE = 0,
//    ATTACK_TYPE_DOS = 1,
//    ATTACK_TYPE_SYBIL = 2,
//    ATTACK_TYPE_FALSIFIED = 3
//};
//
//class CarApp : public ApplicationBase
//{
//  protected:
//    // ⭐⭐ ADD THIS LINE ⭐⭐
//    IInterfaceTable *interfaceTable = nullptr;
//
//    // Configuration
//    int mySenderId = -1;
//    int seqNumber = 0;
//    int getWlanInterfaceId();
//
//    // Mobility data
//    std::string mobilityFile;
//    std::vector<std::pair<double, std::pair<double, double>>> mobilityData;
//    size_t mobilityDataIndex = 0;
//
//    double currentX = 0.0;
//    double currentY = 0.0;
//    double currentZ = 0.0;
//    double currentSpeed = 0.0;
//    double currentDirection = 0.0;
//
//    // Attack schedule
//    struct AttackEvent {
//        double time;
//        int attackType;
//    };
//    std::vector<AttackEvent> attackSchedule;
//    bool isAttacking = false;
//    int currentAttackType = ATTACK_TYPE_NONE;
//
//    // Attack parameters
//    double attackSeverity = 1.0;
//    std::vector<int> fakeIds;
//
//    // Statistics
//    cOutVector packetSentVector;
//    cOutVector attackRateVector;
//    cOutVector positionXVector;
//    cOutVector positionYVector;
//    cOutVector speedVector;
//
//    // Timers
//    cMessage *mobilityUpdateMsg = nullptr;
//    cMessage *sendMessageMsg = nullptr;
//    cMessage *attackMsg = nullptr;
//
//  public:
//    CarApp() {}
//    virtual ~CarApp();
//
//  protected:
//    virtual void initialize(int stage) override;
//    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
//    virtual void handleMessageWhenUp(cMessage *msg) override;
//    virtual void finish() override;
//
//    virtual void handleStartOperation(LifecycleOperation *operation) override;
//    virtual void handleStopOperation(LifecycleOperation *operation) override;
//    virtual void handleCrashOperation(LifecycleOperation *operation) override;
//
//    // Mobility
//    void loadMobilityData();
//    void updateMobility();
//
//    // Attack
//    void loadAttackSchedule(const std::string& filename);
//    void checkForAttack();
//    void startAttack(int attackType);
//    void stopAttack();
//
//    // Communication
//    void sendNormalMessage();
//    void sendDoSAttack();
//    void sendSybilAttack();
//    void sendFalsifiedData();
//    void processReceivedPacket(Packet *packet);
//    void sendToNic(Packet *packet);
//};
//
//#endif





//
//#ifndef __CARAPP_H_
//#define __CARAPP_H_
//
//#include <omnetpp.h>
//#include <string>
//#include <fstream>
//#include <vector>
//#include <map>
//#include "inet/common/INETDefs.h"
//#include "inet/common/TimeTag_m.h"
//#include "inet/transportlayer/contract/udp/UdpSocket.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/networklayer/contract/ipv4/Ipv4Address.h"
//
//using namespace omnetpp;
//using namespace inet;
//
//// Define attack types
//enum {
//    ATTACK_TYPE_NONE = 0,
//    ATTACK_TYPE_DOS = 1,      // Denial of Service
//    ATTACK_TYPE_SYBIL = 2,    // Sybil attack (fake IDs)
//    ATTACK_TYPE_FALSIFIED = 3 // Falsified data
//};
//
///**
// * Application for Car to Car (V2V) communication
// * Supports normal messages and attack simulation
// */
//class CarApp : public cSimpleModule
//{
//  protected:
//    // Configuration
//    int mySenderId = -1;        // ID of this car
//    int seqNumber = 0;          // Sequence number for packets
//
//    // Attack parameters
//    bool isAttacker = false;    // Is this car an attacker?
//    int myAttackType = ATTACK_TYPE_NONE;
//    double myAttackSeverity = 0.0;
//    simtime_t myAttackStartTime;
//    simtime_t myAttackEndTime;
//    std::vector<int> fakeIds;   // For Sybil attack
//
//    // Statistics
//    cOutVector packetSentVector;
//    cOutVector attackRateVector;
//
//    // Timers
//    cMessage *periodicMsgEvent = nullptr;
//    cMessage *attackMsgEvent = nullptr;
//
//    // Network related
//    UdpSocket socket;           // UDP socket for communication
//    L3Address broadcastAddr;    // Multicast address for broadcast
//    int broadcastPort;          // Port for broadcast messages
//
//  public:
//    // Constructor/destructor
//    virtual ~CarApp() {
//        cancelAndDelete(periodicMsgEvent);
//        cancelAndDelete(attackMsgEvent);
//    }
//
//  protected:
//    // OMNeT++ module methods
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//
//    // Helper methods
//    void loadAttackNodesFromCSV(const std::string& filename);
//    void startAttack();
//    void stopAttack();
//
//    // Message sending methods
//    void sendNormalMessage();
//    void sendDoSAttack();
//    void sendSybilAttack();
//    void sendFalsifiedData();
//};
//
//#endif



//#ifndef __RDOEXPERIMENT_CARAPP_H_
//#define __RDOEXPERIMENT_CARAPP_H_
//
//#include <omnetpp.h>
//#include <fstream>
//#include <string>
//#include <map>
//#include <vector>
//#include "inet/common/packet/Packet.h"
//#include "inet/networklayer/common/L3Address.h"
//#include "inet/networklayer/common/L3AddressResolver.h"
//#include "inet/networklayer/common/L3AddressTag_m.h"
//#include "inet/common/TimeTag_m.h"
//
//using namespace omnetpp;
//using namespace inet;
//
//// Define attack types
//#define ATTACK_TYPE_NONE      0
//#define ATTACK_TYPE_DOS       1
//#define ATTACK_TYPE_SYBIL     2
//#define ATTACK_TYPE_FALSIFIED 3
//
///**
// * Car application for simulating normal and attack behavior
// */
//class CarApp : public cSimpleModule
//{
//  private:
//    // Configuration
//    int mySenderId;
//    bool isAttacker = false;
//    int myAttackType = ATTACK_TYPE_NONE;
//    double myAttackSeverity = 0.0;
//    simtime_t myAttackStartTime;
//    simtime_t myAttackEndTime;
//    std::vector<int> fakeIds;
//    int seqNumber = 0;
//
//    // Scheduled events
//    cMessage *periodicMsgEvent = nullptr;
//    cMessage *attackMsgEvent = nullptr;
//
//    // Statistics
//    cOutVector packetSentVector;
//    cOutVector attackRateVector;
//
//  public:
//    virtual ~CarApp() {
//        cancelAndDelete(periodicMsgEvent);
//        cancelAndDelete(attackMsgEvent);
//    }
//
//  protected:
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//
//    // Helper methods
//    void loadAttackNodesFromCSV(const std::string& filename);
//    void sendNormalMessage();
//    void startAttack();
//    void stopAttack();
//    void sendDoSAttack();
//    void sendSybilAttack();
//    void sendFalsifiedData();
//};
//
//#endif
