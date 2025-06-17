

#include "CarApp.h"
#include "RDOAgent.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/common/socket/SocketTag_m.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "inet/common/checksum/TcpIpChecksum.h"
#include "inet/common/DirectionTag_m.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/Protocol.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/common/L4PortTag_m.h"
#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
#include "inet/transportlayer/udp/Udp.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/common/InterfaceTable.h"

using namespace inet;
using namespace omnetpp;
Define_Module(CarApp);

CarApp::~CarApp()
{
    cancelAndDelete(mobilityUpdateMsg);
    cancelAndDelete(sendMessageMsg);
    cancelAndDelete(attackMsg);
    if (rdoEnabled) {
        cancelAndDelete(rdoUpdateEvent);
        delete rdoAgent;
    }
}

void CarApp::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        // Initialize output vectors
        packetSentVector.setName("packetsSent");
        attackRateVector.setName("attackRate");
        positionXVector.setName("positionX");
        positionYVector.setName("positionY");
        speedVector.setName("speed");
        rdoStateVector.setName("rdoState");
        rdoGradientVector.setName("rdoGradient");
        rdoConsensusVector.setName("rdoConsensus");

        // Register signals
        sinrSignal = registerSignal("sinr");
        packetSentSignal = registerSignal("packetSent");
        packetReceivedSignal = registerSignal("packetReceived");
        endToEndDelaySignal = registerSignal("endToEndDelay");

        // Extract Car ID from module name (e.g., car[5])
        std::string fullName = getParentModule()->getFullName();
        size_t start = fullName.find('[') + 1;
        size_t end = fullName.find(']');
        if (start != std::string::npos && end != std::string::npos) {
            std::string idStr = fullName.substr(start, end - start);
            try {
                mySenderId = std::stoi(idStr);
            } catch (const std::exception& e) {
                EV_ERROR << "Error parsing car ID: " << e.what() << endl;
                mySenderId = 0;
            }
        } else {
            EV_ERROR << "Cannot extract car ID from module name " << fullName << endl;
            mySenderId = 0;
        }

        // Get parameters
        mobilityFile = par("mobilityFile").stringValue();

        // Check if RDO is enabled
        rdoEnabled = getSystemModule()->par("useRDO").boolValue();
    }
    else if (stage == INITSTAGE_NETWORK_LAYER) {
        // Find Interface Table
        interfaceTable = findModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        if (!interfaceTable)
            throw cRuntimeError("InterfaceTable not found! Check 'interfaceTableModule' parameter.");

        EV_INFO << "Interface Table initialized with " << interfaceTable->getNumInterfaces() << " interfaces." << endl;
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        // Check if the node is up
        NodeStatus* nodeStatus = dynamic_cast<NodeStatus*>(getContainingNode(this)->getSubmodule("status"));
        bool isNodeUp = !nodeStatus || nodeStatus->getState() == NodeStatus::UP;
        if (!isNodeUp)
            throw cRuntimeError("Application is not ready (node DOWN)");

        // Load mobility data
        if (!mobilityFile.empty()) {
            loadMobilityData();
        }

        // Load attack schedule if enabled
        bool useAttackSchedule = getSystemModule()->par("useAttackSchedule").boolValue();
        if (useAttackSchedule) {
            std::string attackFile = getSystemModule()->par("attackScheduleFile").stringValue();
            if (!attackFile.empty()) {
                loadAttackSchedule(attackFile);
            }
        }

        // Create timers/messages
        mobilityUpdateMsg = new cMessage("mobilityUpdate");
        sendMessageMsg = new cMessage("sendMessage");
        attackMsg = new cMessage("attackControl");

        // Schedule first events
        scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
        scheduleAt(simTime() + 1.0, sendMessageMsg);

        // If RDO is enabled, initialize RDO agent
        if (rdoEnabled) {
            loadRDOConfig("src/rdoexperiment/data/Phase3/rdo_config.csv");
            rdoUpdateEvent = new cMessage("rdoUpdate");
            initializeRDO();
            scheduleAt(simTime() + 5.0, rdoUpdateEvent);  // allow time for neighbors to respond
        }
    }
    else if (stage == INITSTAGE_TRANSPORT_LAYER) {
        // UDP Socket initialization with improved error handling
        socket.setOutputGate(gate("socketOut"));
        socket.bind(10000 + mySenderId);  // Unique port based on Car ID

        // Enable broadcast capability
        socket.setBroadcast(true);

        // Try to resolve gNB address for unicast
        L3Address destAddr;
        try {
            // Try direct connection to gNB - distribute cars among gNBs
            std::string gnbAddr = "gnb[" + std::to_string(mySenderId % 3) + "]";
            std::string fullPath = std::string("gnb[") + std::to_string(mySenderId % 3) + "]";
            cModule* gnbModule = getSystemModule()->getSubmodule("gnb", mySenderId % 3);
            if (gnbModule) {
                L3AddressResolver resolver;
                destAddr = resolver.addressOf(gnbModule);
                EV_WARN << "Resolved gNB address for " << fullPath << ": " << destAddr.str() << endl;
            } else {
                destAddr = L3Address();
                EV_ERROR << "Could not find gNB module: " << fullPath << endl;
            }
            EV_WARN << "Resolved gNB address: " << gnbAddr << " to " << destAddr.str() << endl;
        } catch (const std::exception& e) {
            destAddr = L3Address();
            EV_ERROR << "Failed to resolve gNB address: " << e.what() << endl;
        }

        if (destAddr.isUnspecified()) {
            // Fall back to broadcast if unicast fails
            EV_WARN << "Using broadcast address as fallback" << endl;
            socket.connect(L3AddressResolver().resolve("255.255.255.255"), 9000);
        } else {
            EV_WARN << "Car[" << mySenderId << "] connecting to gNB at " << destAddr.str() << endl;
            socket.connect(destAddr, 9000);
        }

        EV_WARN << "UDP socket bound to port " << (10000 + mySenderId)
                << " and connected to destination port 9000" << endl;

        // Check connection to gNB
        cModule* cellularNic = getParentModule()->getSubmodule("cellularNic");
        if (cellularNic) {
            EV_WARN << "Car[" << mySenderId << "] cellularNic module found" << endl;
            cModule* phy = cellularNic->getSubmodule("phy");
            if (phy) {
                EV_WARN << "Car[" << mySenderId << "] PHY module found" << endl;
            }
        }
    }
}

void CarApp::loadRDOConfig(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        EV_WARN << "Could not open RDO config file: " << filename << endl;
        return;
    }

    std::string line;
    getline(file, line); // Skip header

    while (getline(file, line)) {
        std::istringstream ss(line);
        std::string token;

        // Parse node_id
        if (!getline(ss, token, ',')) continue;
        int nodeId = std::stoi(token);

        if (nodeId != mySenderId)
            continue; // Only care about our own line

        // Parse trust_score (ignored here)
        if (!getline(ss, token, ',')) continue;

        // Parse filter_type
        if (!getline(ss, token, ',')) continue;
        loadedFilterType = std::stoi(token);

        // Parse consensus_algorithm
        if (!getline(ss, token, ',')) continue;
        loadedConsensusAlgorithm = std::stoi(token);

        EV_INFO << "Loaded RDO config for node " << mySenderId
                << ": filterType=" << loadedFilterType
                << ", consensusAlgorithm=" << loadedConsensusAlgorithm << endl;
        break;
    }
}

void CarApp::handleMessageWhenUp(cMessage* msg)
{
    if (msg->isSelfMessage()) {
        if (msg == mobilityUpdateMsg) {
            updateMobility();
            scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
        }
        else if (msg == sendMessageMsg) {
            checkForAttack();
            if (isAttacking && currentAttackType == ATTACK_TYPE_DOS) {
                sendDoSAttack();
            } else {
                sendNormalMessage();
            }

            scheduleAt(simTime() + exponential(1.0), sendMessageMsg);
        }
        else if (msg == rdoUpdateEvent) {
            if (rdoEnabled) {
                processRDOUpdate();
                scheduleAt(simTime() + 0.5, rdoUpdateEvent);
            }
        }
        else if (msg == attackMsg) {
            // attack control if needed
        }
    }
    else {
        if (Packet* packet = dynamic_cast<Packet*>(msg)) {
            // Fix the RDO state packet detection
            const char* packetName = packet->getName();
            bool isRdoPacket = packetName && strncmp(packetName, "RDOState_", 9) == 0;

            if (rdoEnabled && isRdoPacket) {
                EV_WARN << "Car[" << mySenderId << "] received RDO state packet: "
                        << packetName << endl;
                processRDOStatePacket(packet);
                delete msg;
            } else {
                processReceivedPacket(packet);
                delete msg;
            }
        }
        else
            delete msg;
    }
}

void CarApp::finish()
{
    ApplicationBase::finish();
}

void CarApp::handleStartOperation(LifecycleOperation *operation)
{
    // Begin periodic mobility updates and message sending
    if (!mobilityUpdateMsg) {
        mobilityUpdateMsg = new cMessage("mobilityUpdate");
        scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
    }
    if (!sendMessageMsg) {
        sendMessageMsg = new cMessage("sendMessage");
        scheduleAt(simTime() + 1.0, sendMessageMsg);
    }
}

void CarApp::handleStopOperation(LifecycleOperation *operation)
{
    // Stop timers
    cancelEvent(mobilityUpdateMsg);
    cancelEvent(sendMessageMsg);
    cancelEvent(attackMsg);
}

void CarApp::handleCrashOperation(LifecycleOperation *operation)
{
    // Same as stop but abrupt
    handleStopOperation(operation);
}

void CarApp::loadMobilityData()
{
    // Try to open the file with absolute path
    std::string absolutePath = std::string("/home/simu5g/simu5g-workspace/simu5G/src/rdoexperiment/data/Phase3/") +
                               mobilityFile;
    std::ifstream file(absolutePath);

    // If absolute path fails, try relative path
    if (!file.is_open()) {
        std::string relativePath = std::string("src/rdoexperiment/data/Phase3/") + mobilityFile;
        file.open(relativePath);

        // If that also fails, try another relative path
        if (!file.is_open()) {
            std::string anotherPath = std::string("../src/rdoexperiment/data/Phase3/") + mobilityFile;
            file.open(anotherPath);
        }
    }

    if (!file.is_open()) {
        EV_ERROR << "Failed to open mobility file: " << mobilityFile << endl;
        EV_ERROR << "Tried paths: " << absolutePath << " and relative paths" << endl;
        return;
    }

    EV_INFO << "Loading mobility data from file for car " << mySenderId << endl;

    // Read and parse the file
    std::string line;

    // Skip the header if present (first line contains column names)
    getline(file, line);

    // Read data lines
    while (getline(file, line)) {
        std::istringstream ss(line);
        double time, x, y;

        // Attempt to parse time, x, y
        if (ss >> time >> x >> y) {
            mobilityData.push_back({time, {x, y}});
        }
    }

    // Sort data by timestamp if needed
    std::sort(mobilityData.begin(), mobilityData.end());

    EV_INFO << "Loaded " << mobilityData.size() << " mobility points" << endl;

    // Initialize with the first position
    if (!mobilityData.empty()) {
        currentX = mobilityData[0].second.first;
        currentY = mobilityData[0].second.second;
        currentZ = 0.0;

        EV_INFO << "Initial position: (" << currentX << ", " << currentY << ", " << currentZ << ")" << endl;
    }
}

void CarApp::updateMobility()
{
    if (mobilityData.empty()) {
        // No mobility data, use fixed position
        return;
    }

    // Current simulation time
    double now = simTime().dbl();

    // Find the position for the current time
    // This is a simple linear interpolation between waypoints
    while (mobilityDataIndex < mobilityData.size() - 1 &&
           mobilityData[mobilityDataIndex + 1].first <= now) {
        mobilityDataIndex++;
    }

    if (mobilityDataIndex >= mobilityData.size() - 1) {
        // We've reached the end of the mobility data
        currentX = mobilityData.back().second.first;
        currentY = mobilityData.back().second.second;
        currentZ = 0.0;
        currentSpeed = 0.0;
        currentDirection = 0.0;
    } else {
        // Interpolate between current and next waypoint
        double t1 = mobilityData[mobilityDataIndex].first;
        double t2 = mobilityData[mobilityDataIndex + 1].first;

        double x1 = mobilityData[mobilityDataIndex].second.first;
        double y1 = mobilityData[mobilityDataIndex].second.second;

        double x2 = mobilityData[mobilityDataIndex + 1].second.first;
        double y2 = mobilityData[mobilityDataIndex + 1].second.second;

        // Linear interpolation factor
        double alpha = (now - t1) / (t2 - t1);

        // Interpolated position
        double newX = x1 + alpha * (x2 - x1);
        double newY = y1 + alpha * (y2 - y1);

        // Calculate speed and direction
        double dt = 0.1;  // Time interval (100ms)
        double dx = newX - currentX;
        double dy = newY - currentY;

        // Speed in m/s
        currentSpeed = sqrt(dx*dx + dy*dy) / dt;

        // Direction in radians
        if (dx != 0 || dy != 0) {
            currentDirection = atan2(dy, dx);
        }

        // Update position
        currentX = newX;
        currentY = newY;
        currentZ = 0.0;
    }

    // Record position and speed statistics
    positionXVector.record(currentX);
    positionYVector.record(currentY);
    speedVector.record(currentSpeed);

    EV_INFO << "Updated position: (" << currentX << ", " << currentY << ", " << currentZ << "), "
            << "speed: " << currentSpeed << " m/s, direction: " << currentDirection << " rad" << endl;
}

void CarApp::loadAttackSchedule(const std::string& filename)
{
    // Try to open the file with absolute path first
    std::string absolutePath = std::string("/home/simu5g/simu5g-workspace/simu5G//src/rdoexperiment/data/Phase3/") + filename;
    std::ifstream file(absolutePath);

    // If absolute path fails, try relative path
    if (!file.is_open()) {
        file.open(filename);
    }

    if (!file.is_open()) {
        EV_ERROR << "Failed to open attack schedule file: " << filename << endl;
        return;
    }

    EV_INFO << "Loading attack schedule from " << filename << endl;

    // Read and parse the file
    std::string line;

    // Skip the header (first line contains column names)
    getline(file, line);

    // Read attack schedule lines
    while (getline(file, line)) {
        std::istringstream ss(line);
        std::string token;

        // Parse the CSV line
        // Format: sendTime,sender,attackType

        // Parse sendTime
        if (!getline(ss, token, ',')) continue;
        double attackTime;
        try {
            attackTime = std::stod(token);
        } catch (const std::exception&) {
            continue;  // Skip invalid lines
        }

        // Parse sender
        if (!getline(ss, token, ',')) continue;
        int attackerId;
        try {
            attackerId = std::stoi(token);
        } catch (const std::exception&) {
            continue;  // Skip invalid lines
        }

        // Parse attackType
        if (!getline(ss, token, ',')) continue;
        int aType;

        // Map attack type string to enum
        if (token == "DoS" || token == "dos" || token == "DOS") {
            aType = ATTACK_TYPE_DOS;
        } else {
            try {
                aType = std::stoi(token);
                // Only accept DoS attacks (type 1)
                if (aType != ATTACK_TYPE_DOS) {
                    continue;  // Skip unsupported attack types
                }
            } catch (const std::exception&) {
                continue;  // Skip invalid lines
            }
        }

        // Only add attack events for this car
        if (attackerId == mySenderId) {
            AttackEvent event;
            event.time = attackTime;
            event.attackType = aType;
            attackSchedule.push_back(event);
        }
    }

    // Sort attack schedule by time
    std::sort(attackSchedule.begin(), attackSchedule.end(),
        [](const AttackEvent& a, const AttackEvent& b) {
            return a.time < b.time;
        });

    EV_INFO << "Loaded " << attackSchedule.size() << " DoS attack events for car ID " << mySenderId << endl;
}

void CarApp::checkForAttack()
{
    // Current simulation time
    double now = simTime().dbl();

    // Check for attack start or end
    for (const auto& event : attackSchedule) {
        if (event.time <= now && !isAttacking) {
            // Only start DoS attacks, ignore other attack types
            if (event.attackType == ATTACK_TYPE_DOS) {
                startAttack(event.attackType);
            }
            break;
        }
        else if (isAttacking && event.time > now) {
            // End current attack
            stopAttack();
            break;
        }
    }
}

void CarApp::startAttack(int attackType)
{
    if (!isAttacking) {
        isAttacking = true;
        currentAttackType = attackType;

        // Only DoS attacks are supported now
        if (currentAttackType == ATTACK_TYPE_DOS) {
            EV_INFO << "Starting DoS attack at " << simTime() << endl;
        } else {
            // Reset attack status for unsupported attack types
            isAttacking = false;
            currentAttackType = ATTACK_TYPE_NONE;
            EV_INFO << "Attack type " << attackType << " not supported, ignoring at " << simTime() << endl;
        }
    }
}

void CarApp::stopAttack()
{
    if (isAttacking) {
        EV_INFO << "Stopping attack at " << simTime() << endl;
        isAttacking = false;
        currentAttackType = ATTACK_TYPE_NONE;
    }
}

void CarApp::sendNormalMessage()
{
    // Create a new packet
    std::string packetName = "Car_" + std::to_string(mySenderId) + "_" + std::to_string(seqNumber++);
    auto packet = new Packet(packetName.c_str());

    // Create payload with vehicle data
    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> data(100, 0);  // 100 byte packet

    // Include sender ID and message type
    data[0] = mySenderId & 0xFF;
    data[1] = (mySenderId >> 8) & 0xFF;
    data[2] = 0x00;  // Normal message type

    // Include position data
    uint32_t xInt = static_cast<uint32_t>(currentX * 100.0);  // Centimeter precision
    uint32_t yInt = static_cast<uint32_t>(currentY * 100.0);

    data[3] = (xInt >> 0) & 0xFF;
    data[4] = (xInt >> 8) & 0xFF;
    data[5] = (xInt >> 16) & 0xFF;
    data[6] = (xInt >> 24) & 0xFF;

    data[7] = (yInt >> 0) & 0xFF;
    data[8] = (yInt >> 8) & 0xFF;
    data[9] = (yInt >> 16) & 0xFF;
    data[10] = (yInt >> 24) & 0xFF;

    // Include speed and direction
    uint16_t speedInt = static_cast<uint16_t>(currentSpeed * 100.0);  // Centimeter/s precision
    uint16_t dirInt = static_cast<uint16_t>((currentDirection + M_PI) * 1000.0); // Milliradian precision

    data[11] = (speedInt >> 0) & 0xFF;
    data[12] = (speedInt >> 8) & 0xFF;

    data[13] = (dirInt >> 0) & 0xFF;
    data[14] = (dirInt >> 8) & 0xFF;

    // Set the data in the payload
    payload->setBytes(data);
    packet->insertAtBack(payload);

    // Add timestamp tag
    packet->addTag<CreationTimeTag>()->setCreationTime(simTime());

    // Add protocol tag (needed by the message dispatcher)
    packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);

    // Log before sending
    EV_WARN << "Preparing to send normal message: " << packet->getName() << endl;

    // Send the packet
    sendToNic(packet);

    // Emit signal for metrics collection
    emit(packetSentSignal, 1.0);
    getParentModule()->emit(packetSentSignal, 1.0);  // Also emit at parent level for better propagation

    // Update statistics
    packetSentVector.record(1);
}

void CarApp::sendDoSAttack()
{
    // DoS attack sends multiple packets in a short time
    int numPackets = par("dosPacketCount");

    for (int i = 0; i < numPackets; i++) {
        // Create a packet similar to normal but with attack marker
        std::string packetName = "Car_" + std::to_string(mySenderId) + "_DoS_" + std::to_string(seqNumber++);
        auto packet = new Packet(packetName.c_str());

        // Create larger payload to consume bandwidth
        auto payload = makeShared<BytesChunk>();
        std::vector<uint8_t> data(500, 0);  // 500 byte packet (5x normal)

        // Include sender ID and message type
        data[0] = mySenderId & 0xFF;
        data[1] = (mySenderId >> 8) & 0xFF;
        data[2] = 0x01;  // DoS attack marker

        // Set the data in the payload
        payload->setBytes(data);
        packet->insertAtBack(payload);

        // Add timestamp tag
        packet->addTag<CreationTimeTag>()->setCreationTime(simTime());

        // Add protocol tag (needed by the message dispatcher)
        packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);

        // Send the packet
        sendToNic(packet);
    }

    // Emit total packets sent in this DoS attack
    emit(packetSentSignal, (double)numPackets);
    getParentModule()->emit(packetSentSignal, (double)numPackets);

    // Update statistics
    attackRateVector.record(numPackets);
}
void CarApp::processReceivedPacket(Packet *packet)
{
    // Process incoming packets (e.g., from other vehicles)
    EV_WARN << "PACKET RECEIVED by Car[" << mySenderId << "]: " << packet->getName() << " at time " << simTime() << endl;

    // Extract data from the packet
    auto payload = packet->peekData<BytesChunk>();
    if (payload) {
        // Process the data
        auto& bytes = payload->getBytes();

        if (bytes.size() >= 3) {
            // Extract sender ID
            int senderId = bytes[0] | (bytes[1] << 8);

            // Extract message type
            int messageType = bytes[2];

            // Extract position if available
            double posX = 0.0, posY = 0.0;
            if (bytes.size() >= 11) {
                uint32_t xInt = bytes[3] | (bytes[4] << 8) | (bytes[5] << 16) | (bytes[6] << 24);
                uint32_t yInt = bytes[7] | (bytes[8] << 8) | (bytes[9] << 16) | (bytes[10] << 24);

                posX = xInt / 100.0;  // Convert from centimeters to meters
                posY = yInt / 100.0;
            }

            // Extract speed and direction if available
            double speed = 0.0, direction = 0.0;
            if (bytes.size() >= 15) {
                uint16_t speedInt = bytes[11] | (bytes[12] << 8);
                uint16_t dirInt = bytes[13] | (bytes[14] << 8);

                speed = speedInt / 100.0;     // Convert from cm/s to m/s
                direction = (dirInt / 1000.0) - M_PI;  // Convert from milliradians and shift range
            }

            // Process based on message type
            std::string msgTypeStr;
            switch (messageType) {
                case 0x00:
                    msgTypeStr = "Normal";
                    break;
                case 0x01:
                    msgTypeStr = "DoS";
                    break;
                default:
                    msgTypeStr = "Unknown";
                    break;
            }

            EV_INFO << "  - Sender ID: " << senderId << endl;
            EV_INFO << "  - Message Type: " << msgTypeStr << endl;
            EV_INFO << "  - Position: (" << posX << ", " << posY << ")" << endl;
            EV_INFO << "  - Speed: " << speed << " m/s, Direction: " << direction << " rad" << endl;

            // Emit that a packet was successfully received
            emit(packetReceivedSignal, 1.0);
            getParentModule()->emit(packetReceivedSignal, 1.0);  // Also emit at parent level

            // Calculate and emit end-to-end delay if timestamp is available
            auto timeTag = packet->findTag<CreationTimeTag>();
            if (timeTag) {
                simtime_t delay = simTime() - timeTag->getCreationTime();
                EV_INFO << "  - End-to-end delay: " << delay << " s" << endl;
                emit(endToEndDelaySignal, delay.dbl());
            }
        }
    }
}

void CarApp::initializeRDO()
{
    EV_INFO << "car[" << mySenderId << "] initializing RDO" << endl;

    // Check if forceTrusted parameter exists
    try {
        forceTrusted = par("forceTrusted").boolValue();
    } catch (const std::exception& e) {
        // Parameter doesn't exist, use default (false)
        forceTrusted = false;
    }

    // Determine trust status with multiple options
    // 1. gNBs with ID >= 1000 are trusted
    // 2. Nodes with the forceTrusted parameter
    // 3. Nodes with IDs 0, 1, 2 (to match gNBs in ini file)
    isTrusted = (mySenderId >= 1000) || forceTrusted || (mySenderId == 0 || mySenderId == 1 || mySenderId == 2);

    EV_WARN << "Node " << mySenderId << " trust status: " << (isTrusted ? "TRUSTED" : "NOT TRUSTED") << endl;

    // Initialize cost function parameters - consistent with both implementations
    costFunctionTarget = 50.0 + mySenderId * 10.0;
    costFunctionWeight = 1.0 + (mySenderId % 3) * 0.2;

    // Define trusted gNB IDs (from original implementation)
    trustedGnbIds = {1000, 1001, 1002};

    // Also add some cars as trusted (for initial trust setup)
    for (int i = 0; i < 4; i++) {
        if (std::find(trustedGnbIds.begin(), trustedGnbIds.end(), i) == trustedGnbIds.end()) {
            trustedGnbIds.push_back(i);
        }
    }

    // Initialize tracking metrics
    lastReceivedTime = 0.0;
    totalRdoPacketsReceived = 0;
    totalRdoPacketsSent = 0;

    // Create RDO agent with sender ID and trust status
    rdoAgent = new RDOAgent(mySenderId, isTrusted);

    // Set filter type from configuration (combining both approaches)
    // Using the loaded configuration if available, otherwise default to type 3
    int filterType = (loadedFilterType > 0) ? loadedFilterType : 3;
    rdoAgent->setFilterType(filterType);

    // Set consensus algorithm from configuration
    int consensusAlgo = (loadedConsensusAlgorithm > 0) ? loadedConsensusAlgorithm : 1;
    rdoAgent->setConsensusAlgorithm(consensusAlgo);

    // Set cost function target and max state change in agent
    rdoAgent->setCostFunctionTarget(costFunctionTarget);
    rdoAgent->setMaxStateChange(10.0);  // Limit state changes for stability

    // Build neighbor list (all cars + trusted gNBs)
    std::vector<int> allNeighbors;
    int numCars = getSystemModule()->par("numCars").intValue();

    // Add all cars except self
    for (int i = 0; i < numCars; i++) {
        if (i != mySenderId) {
            allNeighbors.push_back(i);
        }
    }

    // Add all trusted gNBs
    for (int gnbId : trustedGnbIds) {
        if (gnbId >= 1000) {  // Only add actual gNBs here
            allNeighbors.push_back(gnbId);  // Keep original IDs (1000+)
        }
    }

    // Initialize RDO agent state using position as initial state
    // IMPORTANT FIX: Ensure non-zero initial state
    double initialState = currentX;

    // Check if initial state is zero or too small
    if (std::abs(initialState) < 0.1) {
        // Force a non-zero initial state based on node ID
        initialState = 10.0 + mySenderId * 5.0;
        std::cout << "FORCING NON-ZERO INITIAL STATE " << initialState
                  << " for node " << mySenderId << std::endl;
    }

    rdoAgent->initialize(initialState, trustedGnbIds, allNeighbors);

    // Calculate initial gradient and set it
    double gradient = generateGradient();
    rdoAgent->setGradient(gradient);

    // Record initial state and gradient
    rdoStateVector.record(initialState);
    rdoGradientVector.record(gradient);

    // Only trusted nodes broadcast their states initially
    if (isTrusted) {
        sendRDOState();
    }

    EV_INFO << "Initialized RDO agent for node " << mySenderId
            << " with filterType=" << filterType
            << ", consensusAlgo=" << consensusAlgo
            << ", initialState=" << initialState
            << ", isTrusted=" << isTrusted
            << ", neighbors=" << allNeighbors.size() << endl;

    // Direct console output for more visibility
    std::cout << "Initialized RDO agent for node " << mySenderId
              << " with initialState=" << initialState
              << ", target=" << costFunctionTarget
              << ", gradient=" << gradient << std::endl;
}

void CarApp::processRDOUpdate()
{
    if (!rdoAgent) {
        EV_ERROR << "RDO agent is not initialized in processRDOUpdate" << endl;
        return;
    }

    // Log the current state of neighborStates
    EV_WARN << "Car[" << mySenderId << "] updating RDO state at time " << simTime()
            << ". neighborStates size: " << neighborStates.size()
            << ", total packets received: " << totalRdoPacketsReceived
            << ", total packets sent: " << totalRdoPacketsSent << endl;

    // Generate gradient based on local cost function
    double gradient = generateGradient();
    rdoAgent->setGradient(gradient);

    // Update step size according to the paper: αk decreases with iterations
    double iterationNumber = simTime().dbl() / 0.5;

    // Use a larger step size initially
    double stepSize;
    if (iterationNumber < 5.0) {
        // Use larger step size for first few iterations
        stepSize = 0.5;
    } else {
        // Then switch to decreasing step size
        stepSize = 1.0 / (iterationNumber + 1.0);
    }

    std::cout << "Step size for node " << mySenderId << " at time " << simTime().dbl()
              << " (iteration " << iterationNumber << "): " << stepSize << std::endl;

    // Get current state for comparison
    double oldState = rdoAgent->getState();

    // IMPORTANT CHANGE: Use consensus-based update method instead of projection-based
    // This enhances communication between nodes
    double newState;

    // Alternate between update methods based on node ID for testing both approaches
    if (mySenderId % 2 == 0) {
        // Even IDs use pure projection to maintain strict bounds
        newState = rdoAgent->updateStateWithProjection(neighborStates, stepSize);
        std::cout << "Node " << mySenderId << " using PROJECTION-based update" << std::endl;
    } else {
        // Odd IDs use consensus-based update for more neighbor influence
        newState = rdoAgent->updateState(neighborStates, stepSize);
        std::cout << "Node " << mySenderId << " using CONSENSUS-based update" << std::endl;
    }

    rdoAgent->setState(newState);

    // Record updated state and gradient values
    rdoStateVector.record(newState);
    rdoGradientVector.record(gradient);

    // Calculate and record consensus value
    std::map<int, double> allStates = neighborStates;
    allStates[mySenderId] = newState;

    double consensusValue = 0.0;
    if (!allStates.empty()) {
        for (auto& entry : allStates) {
            consensusValue += entry.second;
            EV_DEBUG << "  Including in consensus: Node " << entry.first
                    << " state=" << entry.second << endl;
        }
        consensusValue /= allStates.size();

        EV_INFO << "Calculated consensus value: " << consensusValue
                << " based on " << allStates.size() << " states" << endl;

        rdoConsensusVector.record(consensusValue);
    } else {
        // If no neighbor states available, use own state
        consensusValue = newState;
        EV_WARN << "No neighbor states available - using own state for consensus: "
                << consensusValue << endl;
        rdoConsensusVector.record(consensusValue);
    }

    // Calculate the value of the local cost function
    double costValue = costFunctionWeight * pow(newState - costFunctionTarget, 2);

    EV_INFO << "Updated state for node " << mySenderId
            << " from " << oldState << " to " << newState
            << " (change: " << (newState - oldState) << ")"
            << " (step size=" << stepSize
            << ", cost value=" << costValue << ")" << endl;

    // Direct console output for visibility
    std::cout << "RDO Update for node " << mySenderId
              << " at time " << simTime().dbl()
              << ": " << oldState << " -> " << newState
              << " (change: " << (newState - oldState) << ")"
              << ", target=" << costFunctionTarget
              << ", gap=" << std::abs(newState - costFunctionTarget) << std::endl;

    // Broadcast state to all neighbors
    sendRDOState();
}
void CarApp::sendRDOState()
{
    if (!rdoAgent) return;

    // Create packet name with sequence number
    std::string packetName = "RDOState_" + std::to_string(mySenderId) + "_" + std::to_string(seqNumber++);
    auto packet = new Packet(packetName.c_str());

    // Create payload chunk
    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> data(100, 0);  // 100 byte packet

    // Include sender ID and RDO message type
    data[0] = mySenderId & 0xFF;
    data[1] = (mySenderId >> 8) & 0xFF;
    data[2] = 0x10;  // RDO state message type

    // Get the current state from the RDO agent
    double stateToSend = rdoAgent->getState();

    // IMPORTANT CHANGE: Implement attack simulation for specific nodes
    // Car[1] will inject false extreme values when attacking
    if (mySenderId == 1 && isAttacking && currentAttackType == ATTACK_TYPE_DOS) {
        // Inject a false extreme value for DoS attacks from Car[1]
        stateToSend = 1000.0; // Extreme value to test outlier rejection
        std::cout << "⚠️ ATTACK: Node " << mySenderId
                  << " injecting false extreme value: " << stateToSend
                  << " instead of actual state: " << rdoAgent->getState() << std::endl;
    }

    // Include state value (double - 8 bytes)
    uint64_t stateBits;
    memcpy(&stateBits, &stateToSend, sizeof(double));
    for (int i = 0; i < 8; i++) {
        data[3 + i] = (stateBits >> (i * 8)) & 0xFF;
    }

    // Get gradient value
    double gradToSend = rdoAgent->getGradient();

    // IMPORTANT CHANGE: Implement another type of attack for Car[2]
    // Car[2] will send false gradient value when attacking
    if (mySenderId == 2 && isAttacking && currentAttackType == ATTACK_TYPE_DOS) {
        // Inject a false gradient to disrupt optimization
        gradToSend = -1000.0; // Extreme gradient to test resilience
        std::cout << "⚠️ ATTACK: Node " << mySenderId
                  << " injecting false gradient: " << gradToSend
                  << " instead of actual gradient: " << rdoAgent->getGradient() << std::endl;
    }

    // Include gradient value (double - 8 bytes)
    uint64_t gradBits;
    memcpy(&gradBits, &gradToSend, sizeof(double));
    for (int i = 0; i < 8; i++) {
        data[11 + i] = (gradBits >> (i * 8)) & 0xFF;
    }

    // Set payload bytes
    payload->setBytes(data);
    packet->insertAtBack(payload);

    // Add tags
    packet->addTag<CreationTimeTag>()->setCreationTime(simTime());
    packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);

    // Log packet sending
    EV_WARN << "Car[" << mySenderId << "] sending RDO state packet #" << totalRdoPacketsSent
            << " at time " << simTime() << ": "
            << packetName << " with state=" << stateToSend
            << ", gradient=" << gradToSend << endl;

    // Use broadcast for RDO state dissemination
    try {
        L3Address broadcastAddr = L3AddressResolver().resolve("255.255.255.255");
        socket.sendTo(packet, broadcastAddr, 9000);
        totalRdoPacketsSent++;

        EV_WARN << "Broadcasting RDO state to " << broadcastAddr.str() << ":9000" << endl;
    } catch (const std::exception& e) {
        EV_ERROR << "Exception in sendRDOState: " << e.what() << endl;
        delete packet;  // Prevent memory leak
    }
}

void CarApp::processRDOStatePacket(Packet* packet)
{
    auto payload = packet->peekData<BytesChunk>();
    if (!payload || payload->getBytes().size() < 19) {
        EV_ERROR << "Invalid RDO state packet: insufficient data" << endl;
        return;
    }

    auto& bytes = payload->getBytes();

    // Extract sender ID
    int senderId = bytes[0] | (bytes[1] << 8);

    // MODIFICATION: Record all packets including untrusted for analysis
    // But mark untrusted packets specially
    bool isTrustedSender = false;

    // Check if sender is in the trusted gNB list (IDs 1000+)
    if (std::find(trustedGnbIds.begin(), trustedGnbIds.end(), senderId) != trustedGnbIds.end()) {
        isTrustedSender = true;
    }

    // Alternative approach: trusted car IDs
    std::vector<int> trustedCarIds = {0, 1, 2, 3};  // All cars are trusted for testing
    if (std::find(trustedCarIds.begin(), trustedCarIds.end(), senderId) != trustedCarIds.end()) {
        isTrustedSender = true;
    }

    // Log the trust status but process all packets
    if (!isTrustedSender) {
        EV_WARN << "Processing RDO state from UNTRUSTED node: " << senderId << endl;
    }

    // Verify message type
    if (bytes[2] != 0x10) {
        EV_ERROR << "Expected RDO state message (0x10), but got " << static_cast<int>(bytes[2]) << endl;
        return;
    }

    // Extract state value
    uint64_t stateInt = 0;
    for (int i = 0; i < 8; i++) {
        stateInt |= static_cast<uint64_t>(bytes[3 + i]) << (i * 8);
    }
    double stateVal;
    memcpy(&stateVal, &stateInt, sizeof(double));

    // Extract gradient value (for logging)
    uint64_t gradientInt = 0;
    for (int i = 0; i < 8; i++) {
        gradientInt |= static_cast<uint64_t>(bytes[11 + i]) << (i * 8);
    }
    double gradientVal;
    memcpy(&gradientVal, &gradientInt, sizeof(double));

    // ENHANCEMENT: Check for implausible values (indicating attack)
    bool isValuePlausible = std::abs(stateVal) < 1000.0;
    bool isGradientPlausible = std::abs(gradientVal) < 1000.0;

    if (!isValuePlausible || !isGradientPlausible) {
        std::cout << "⚠️ IMPLAUSIBLE VALUES DETECTED from node " << senderId
                  << " - State: " << stateVal
                  << ", Gradient: " << gradientVal << std::endl;

        // For attack analysis, still store these values
        // In a production system, you might want to reject them
    }

    // Update neighbor state map (we process all messages, filtering happens in the RDO agent)
    neighborStates[senderId] = stateVal;

    // Update tracking metrics
    lastReceivedTime = simTime().dbl();
    totalRdoPacketsReceived++;

    EV_WARN << "RDO state received from " << (isTrustedSender ? "trusted" : "untrusted") << " node " << senderId
            << ": state=" << stateVal
            << ", gradient=" << gradientVal
            << ", total packets received=" << totalRdoPacketsReceived << endl;

    // Direct console output for attack analysis
    std::cout << "Node " << mySenderId << " received state " << stateVal
              << " from " << (isTrustedSender ? "TRUSTED" : "UNTRUSTED") << " node " << senderId
              << (isValuePlausible && isGradientPlausible ? "" : " [SUSPICIOUS VALUES]") << std::endl;
}
double CarApp::generateGradient()
{
    double state = rdoAgent ? rdoAgent->getState() : currentX;

    // Using a quadratic cost function as described in the paper: fi(x) = wi(x - ai)²
    // where ai is a target value specific to this node
    // and wi is a weight parameter

    // Compute the gradient of the cost function: f'i(x) = 2*wi*(x - ai)
    double gradient = 2.0 * costFunctionWeight * (state - costFunctionTarget);

    // Direct console output
    std::cout << "Generated gradient for node " << mySenderId
              << ": " << gradient
              << " (state=" << state
              << ", target=" << costFunctionTarget
              << ", weight=" << costFunctionWeight << ")" << std::endl;

    EV_INFO << "Generated gradient for node " << mySenderId
            << ": " << gradient
            << " (state=" << state
            << ", target=" << costFunctionTarget
            << ", weight=" << costFunctionWeight << ")" << endl;

    return gradient;
}

int CarApp::getWlanInterfaceId()
{
    // Safe way to get the interface
    if (!interfaceTable)
        throw cRuntimeError("InterfaceTable not initialized properly!");

    // First try to find cellularNic
    NetworkInterface *entry = interfaceTable->findInterfaceByName("cellularNic");

    // If not found, try alternative names
    if (!entry) {
        entry = interfaceTable->findInterfaceByName("wlan0");
    }

    // If still not found, try to get the first wireless interface
    if (!entry) {
        for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
            NetworkInterface *iface = interfaceTable->getInterface(i);
            if (iface && iface->isWireless()) {
                entry = iface;
                EV_INFO << "Using wireless interface: " << iface->getInterfaceName() << endl;
                break;
            }
        }
    }

    // Last resort - use the first interface that's not loopback
    if (!entry) {
        for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
            NetworkInterface *iface = interfaceTable->getInterface(i);
            if (iface && !iface->isLoopback()) {
                entry = iface;
                EV_WARN << "No wireless interface found, using: " << iface->getInterfaceName() << endl;
                break;
            }
        }
    }

    if (!entry)
        throw cRuntimeError("No suitable network interface found!");

    return entry->getInterfaceId();
}

void CarApp::sendToNic(Packet *packet)
{
    try {
        // Add debug info without using unavailable methods
        EV_WARN << "SENDING PACKET: Car[" << mySenderId << "] "
                << "packet: " << packet->getName()
                << " size: " << packet->getByteLength() << " bytes" << endl;

        // Just send using the UDP socket
        socket.send(packet);

        // Log success
        EV_WARN << "Packet sent successfully" << endl;
    } catch (const std::exception& e) {
        EV_ERROR << "Exception in sendToNic: " << e.what() << endl;
        delete packet;  // Prevent memory leak
    }
}




//////WORKING CODE WITH RDO////////////////



//
//#include "CarApp.h"
//#include "RDOAgent.h"
//#include "inet/common/packet/Packet.h"
//#include "inet/common/packet/chunk/BytesChunk.h"
//#include "inet/networklayer/common/L3AddressResolver.h"
//#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
//#include "inet/common/socket/SocketTag_m.h"
//#include "inet/transportlayer/udp/UdpHeader_m.h"
//#include "inet/common/checksum/TcpIpChecksum.h"
//#include "inet/common/DirectionTag_m.h"
//#include "inet/common/ModuleAccess.h"
//#include "inet/common/ProtocolTag_m.h"
//#include "inet/common/Protocol.h"
//#include "inet/common/lifecycle/NodeStatus.h"
//#include "inet/common/lifecycle/ModuleOperations.h"
//#include "inet/linklayer/common/InterfaceTag_m.h"
//#include "inet/networklayer/common/L3AddressTag_m.h"
//#include "inet/transportlayer/common/L4PortTag_m.h"
//#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
//#include "inet/transportlayer/udp/Udp.h"
//#include "inet/linklayer/common/InterfaceTag_m.h"
//#include "inet/networklayer/common/InterfaceTable.h"
//
//using namespace inet;
//using namespace omnetpp;
//Define_Module(CarApp);
//
//CarApp::~CarApp()
//{
//    cancelAndDelete(mobilityUpdateMsg);
//    cancelAndDelete(sendMessageMsg);
//    cancelAndDelete(attackMsg);
//    if (rdoEnabled) {
//        cancelAndDelete(rdoUpdateEvent);
//        delete rdoAgent;
//    }
//}
//
//void CarApp::initialize(int stage)
//{
//    ApplicationBase::initialize(stage);
//
//    if (stage == INITSTAGE_LOCAL) {
//        // Initialize output vectors
//        packetSentVector.setName("packetsSent");
//        attackRateVector.setName("attackRate");
//        positionXVector.setName("positionX");
//        positionYVector.setName("positionY");
//        speedVector.setName("speed");
//        rdoStateVector.setName("rdoState");
//        rdoGradientVector.setName("rdoGradient");
//        rdoConsensusVector.setName("rdoConsensus");
//
//        // Register signals
//        sinrSignal = registerSignal("sinr");
//        packetSentSignal = registerSignal("packetSent");
//        packetReceivedSignal = registerSignal("packetReceived");
//        endToEndDelaySignal = registerSignal("endToEndDelay");
//
//        // Extract Car ID from module name (e.g., car[5])
//        std::string fullName = getParentModule()->getFullName();
//        size_t start = fullName.find('[') + 1;
//        size_t end = fullName.find(']');
//        if (start != std::string::npos && end != std::string::npos) {
//            std::string idStr = fullName.substr(start, end - start);
//            try {
//                mySenderId = std::stoi(idStr);
//            } catch (const std::exception& e) {
//                EV_ERROR << "Error parsing car ID: " << e.what() << endl;
//                mySenderId = 0;
//            }
//        } else {
//            EV_ERROR << "Cannot extract car ID from module name " << fullName << endl;
//            mySenderId = 0;
//        }
//
//        // Get parameters
//        mobilityFile = par("mobilityFile").stringValue();
//
//        // Check if RDO is enabled
//        rdoEnabled = getSystemModule()->par("useRDO").boolValue();
//    }
//    else if (stage == INITSTAGE_NETWORK_LAYER) {
//        // Find Interface Table
//        interfaceTable = findModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
//        if (!interfaceTable)
//            throw cRuntimeError("InterfaceTable not found! Check 'interfaceTableModule' parameter.");
//
//        EV_INFO << "Interface Table initialized with " << interfaceTable->getNumInterfaces() << " interfaces." << endl;
//    }
//    else if (stage == INITSTAGE_APPLICATION_LAYER) {
//        // Check if the node is up
//        NodeStatus* nodeStatus = dynamic_cast<NodeStatus*>(getContainingNode(this)->getSubmodule("status"));
//        bool isNodeUp = !nodeStatus || nodeStatus->getState() == NodeStatus::UP;
//        if (!isNodeUp)
//            throw cRuntimeError("Application is not ready (node DOWN)");
//
//        // Load mobility data
//        if (!mobilityFile.empty()) {
//            loadMobilityData();
//        }
//
//        // Load attack schedule if enabled
//        bool useAttackSchedule = getSystemModule()->par("useAttackSchedule").boolValue();
//        if (useAttackSchedule) {
//            std::string attackFile = getSystemModule()->par("attackScheduleFile").stringValue();
//            if (!attackFile.empty()) {
//                loadAttackSchedule(attackFile);
//            }
//        }
//
//        // Create timers/messages
//        mobilityUpdateMsg = new cMessage("mobilityUpdate");
//        sendMessageMsg = new cMessage("sendMessage");
//        attackMsg = new cMessage("attackControl");
//
//        // Schedule first events
//        scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
//        scheduleAt(simTime() + 1.0, sendMessageMsg);
//
//        // If RDO is enabled, initialize RDO agent
//        if (rdoEnabled) {
//            loadRDOConfig("src/rdoexperiment/data/Phase3/rdo_config.csv");
//            rdoUpdateEvent = new cMessage("rdoUpdate");
//            initializeRDO();
//            scheduleAt(simTime() + 5.0, rdoUpdateEvent);  // allow time for neighbors to respond
//        }
//    }
//    else if (stage == INITSTAGE_TRANSPORT_LAYER) {
//        // UDP Socket initialization with improved error handling
//        socket.setOutputGate(gate("socketOut"));
//        socket.bind(10000 + mySenderId);  // Unique port based on Car ID
//
//        // Enable broadcast capability
//        socket.setBroadcast(true);
//
//        // Try to resolve gNB address for unicast
//        L3Address destAddr;
//        try {
//            // Try direct connection to gNB - distribute cars among gNBs
//            std::string gnbAddr = "gnb[" + std::to_string(mySenderId % 3) + "]";
//            std::string fullPath = std::string("gnb[") + std::to_string(mySenderId % 3) + "]";
//            cModule* gnbModule = getSystemModule()->getSubmodule("gnb", mySenderId % 3);
//            if (gnbModule) {
//                L3AddressResolver resolver;
//                destAddr = resolver.addressOf(gnbModule);
//                EV_WARN << "Resolved gNB address for " << fullPath << ": " << destAddr.str() << endl;
//            } else {
//                destAddr = L3Address();
//                EV_ERROR << "Could not find gNB module: " << fullPath << endl;
//            }
//            EV_WARN << "Resolved gNB address: " << gnbAddr << " to " << destAddr.str() << endl;
//        } catch (const std::exception& e) {
//            destAddr = L3Address();
//            EV_ERROR << "Failed to resolve gNB address: " << e.what() << endl;
//        }
//
//        if (destAddr.isUnspecified()) {
//            // Fall back to broadcast if unicast fails
//            EV_WARN << "Using broadcast address as fallback" << endl;
//            socket.connect(L3AddressResolver().resolve("255.255.255.255"), 9000);
//        } else {
//            EV_WARN << "Car[" << mySenderId << "] connecting to gNB at " << destAddr.str() << endl;
//            socket.connect(destAddr, 9000);
//        }
//
//        EV_WARN << "UDP socket bound to port " << (10000 + mySenderId)
//                << " and connected to destination port 9000" << endl;
//
//        // Check connection to gNB
//        cModule* cellularNic = getParentModule()->getSubmodule("cellularNic");
//        if (cellularNic) {
//            EV_WARN << "Car[" << mySenderId << "] cellularNic module found" << endl;
//            cModule* phy = cellularNic->getSubmodule("phy");
//            if (phy) {
//                EV_WARN << "Car[" << mySenderId << "] PHY module found" << endl;
//            }
//        }
//    }
//}
//
//void CarApp::loadRDOConfig(const std::string& filename)
//{
//    std::ifstream file(filename);
//    if (!file.is_open()) {
//        EV_WARN << "Could not open RDO config file: " << filename << endl;
//        return;
//    }
//
//    std::string line;
//    getline(file, line); // Skip header
//
//    while (getline(file, line)) {
//        std::istringstream ss(line);
//        std::string token;
//
//        // Parse node_id
//        if (!getline(ss, token, ',')) continue;
//        int nodeId = std::stoi(token);
//
//        if (nodeId != mySenderId)
//            continue; // Only care about our own line
//
//        // Parse trust_score (ignored here)
//        if (!getline(ss, token, ',')) continue;
//
//        // Parse filter_type
//        if (!getline(ss, token, ',')) continue;
//        loadedFilterType = std::stoi(token);
//
//        // Parse consensus_algorithm
//        if (!getline(ss, token, ',')) continue;
//        loadedConsensusAlgorithm = std::stoi(token);
//
//        EV_INFO << "Loaded RDO config for node " << mySenderId
//                << ": filterType=" << loadedFilterType
//                << ", consensusAlgorithm=" << loadedConsensusAlgorithm << endl;
//        break;
//    }
//}
//
//void CarApp::handleMessageWhenUp(cMessage* msg)
//{
//    if (msg->isSelfMessage()) {
//        if (msg == mobilityUpdateMsg) {
//            updateMobility();
//            scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
//        }
//        else if (msg == sendMessageMsg) {
//            checkForAttack();
//            if (isAttacking && currentAttackType == ATTACK_TYPE_DOS) {
//                sendDoSAttack();
//            } else {
//                sendNormalMessage();
//            }
//
//            scheduleAt(simTime() + exponential(1.0), sendMessageMsg);
//        }
//        else if (msg == rdoUpdateEvent) {
//            if (rdoEnabled) {
//                processRDOUpdate();
//                scheduleAt(simTime() + 0.5, rdoUpdateEvent);
//            }
//        }
//        else if (msg == attackMsg) {
//            // attack control if needed
//        }
//    }
//    else {
//        if (Packet* packet = dynamic_cast<Packet*>(msg)) {
//            // Fix the RDO state packet detection
//            const char* packetName = packet->getName();
//            bool isRdoPacket = packetName && strncmp(packetName, "RDOState_", 9) == 0;
//
//            if (rdoEnabled && isRdoPacket) {
//                EV_WARN << "Car[" << mySenderId << "] received RDO state packet: "
//                        << packetName << endl;
//                processRDOStatePacket(packet);
//                delete msg;
//            } else {
//                processReceivedPacket(packet);
//                delete msg;
//            }
//        }
//        else
//            delete msg;
//    }
//}
//
//void CarApp::finish()
//{
//    ApplicationBase::finish();
//}
//
//void CarApp::handleStartOperation(LifecycleOperation *operation)
//{
//    // Begin periodic mobility updates and message sending
//    if (!mobilityUpdateMsg) {
//        mobilityUpdateMsg = new cMessage("mobilityUpdate");
//        scheduleAt(simTime() + 0.1, mobilityUpdateMsg);
//    }
//    if (!sendMessageMsg) {
//        sendMessageMsg = new cMessage("sendMessage");
//        scheduleAt(simTime() + 1.0, sendMessageMsg);
//    }
//}
//
//void CarApp::handleStopOperation(LifecycleOperation *operation)
//{
//    // Stop timers
//    cancelEvent(mobilityUpdateMsg);
//    cancelEvent(sendMessageMsg);
//    cancelEvent(attackMsg);
//}
//
//void CarApp::handleCrashOperation(LifecycleOperation *operation)
//{
//    // Same as stop but abrupt
//    handleStopOperation(operation);
//}
//
//void CarApp::loadMobilityData()
//{
//    // Try to open the file with absolute path
//    std::string absolutePath = std::string("/home/simu5g/simu5g-workspace/simu5G/src/rdoexperiment/data/Phase3/") +
//                               mobilityFile;
//    std::ifstream file(absolutePath);
//
//    // If absolute path fails, try relative path
//    if (!file.is_open()) {
//        std::string relativePath = std::string("src/rdoexperiment/data/Phase3/") + mobilityFile;
//        file.open(relativePath);
//
//        // If that also fails, try another relative path
//        if (!file.is_open()) {
//            std::string anotherPath = std::string("../src/rdoexperiment/data/Phase3/") + mobilityFile;
//            file.open(anotherPath);
//        }
//    }
//
//    if (!file.is_open()) {
//        EV_ERROR << "Failed to open mobility file: " << mobilityFile << endl;
//        EV_ERROR << "Tried paths: " << absolutePath << " and relative paths" << endl;
//        return;
//    }
//
//    EV_INFO << "Loading mobility data from file for car " << mySenderId << endl;
//
//    // Read and parse the file
//    std::string line;
//
//    // Skip the header if present (first line contains column names)
//    getline(file, line);
//
//    // Read data lines
//    while (getline(file, line)) {
//        std::istringstream ss(line);
//        double time, x, y;
//
//        // Attempt to parse time, x, y
//        if (ss >> time >> x >> y) {
//            mobilityData.push_back({time, {x, y}});
//        }
//    }
//
//    // Sort data by timestamp if needed
//    std::sort(mobilityData.begin(), mobilityData.end());
//
//    EV_INFO << "Loaded " << mobilityData.size() << " mobility points" << endl;
//
//    // Initialize with the first position
//    if (!mobilityData.empty()) {
//        currentX = mobilityData[0].second.first;
//        currentY = mobilityData[0].second.second;
//        currentZ = 0.0;
//
//        EV_INFO << "Initial position: (" << currentX << ", " << currentY << ", " << currentZ << ")" << endl;
//    }
//}
//
//void CarApp::updateMobility()
//{
//    if (mobilityData.empty()) {
//        // No mobility data, use fixed position
//        return;
//    }
//
//    // Current simulation time
//    double now = simTime().dbl();
//
//    // Find the position for the current time
//    // This is a simple linear interpolation between waypoints
//    while (mobilityDataIndex < mobilityData.size() - 1 &&
//           mobilityData[mobilityDataIndex + 1].first <= now) {
//        mobilityDataIndex++;
//    }
//
//    if (mobilityDataIndex >= mobilityData.size() - 1) {
//        // We've reached the end of the mobility data
//        currentX = mobilityData.back().second.first;
//        currentY = mobilityData.back().second.second;
//        currentZ = 0.0;
//        currentSpeed = 0.0;
//        currentDirection = 0.0;
//    } else {
//        // Interpolate between current and next waypoint
//        double t1 = mobilityData[mobilityDataIndex].first;
//        double t2 = mobilityData[mobilityDataIndex + 1].first;
//
//        double x1 = mobilityData[mobilityDataIndex].second.first;
//        double y1 = mobilityData[mobilityDataIndex].second.second;
//
//        double x2 = mobilityData[mobilityDataIndex + 1].second.first;
//        double y2 = mobilityData[mobilityDataIndex + 1].second.second;
//
//        // Linear interpolation factor
//        double alpha = (now - t1) / (t2 - t1);
//
//        // Interpolated position
//        double newX = x1 + alpha * (x2 - x1);
//        double newY = y1 + alpha * (y2 - y1);
//
//        // Calculate speed and direction
//        double dt = 0.1;  // Time interval (100ms)
//        double dx = newX - currentX;
//        double dy = newY - currentY;
//
//        // Speed in m/s
//        currentSpeed = sqrt(dx*dx + dy*dy) / dt;
//
//        // Direction in radians
//        if (dx != 0 || dy != 0) {
//            currentDirection = atan2(dy, dx);
//        }
//
//        // Update position
//        currentX = newX;
//        currentY = newY;
//        currentZ = 0.0;
//    }
//
//    // Record position and speed statistics
//    positionXVector.record(currentX);
//    positionYVector.record(currentY);
//    speedVector.record(currentSpeed);
//
//    EV_INFO << "Updated position: (" << currentX << ", " << currentY << ", " << currentZ << "), "
//            << "speed: " << currentSpeed << " m/s, direction: " << currentDirection << " rad" << endl;
//}
//
//void CarApp::loadAttackSchedule(const std::string& filename)
//{
//    // Try to open the file with absolute path first
//    std::string absolutePath = std::string("/home/simu5g/simu5g-workspace/simu5G//src/rdoexperiment/data/Phase3/") + filename;
//    std::ifstream file(absolutePath);
//
//    // If absolute path fails, try relative path
//    if (!file.is_open()) {
//        file.open(filename);
//    }
//
//    if (!file.is_open()) {
//        EV_ERROR << "Failed to open attack schedule file: " << filename << endl;
//        return;
//    }
//
//    EV_INFO << "Loading attack schedule from " << filename << endl;
//
//    // Read and parse the file
//    std::string line;
//
//    // Skip the header (first line contains column names)
//    getline(file, line);
//
//    // Read attack schedule lines
//    while (getline(file, line)) {
//        std::istringstream ss(line);
//        std::string token;
//
//        // Parse the CSV line
//        // Format: sendTime,sender,attackType
//
//        // Parse sendTime
//        if (!getline(ss, token, ',')) continue;
//        double attackTime;
//        try {
//            attackTime = std::stod(token);
//        } catch (const std::exception&) {
//            continue;  // Skip invalid lines
//        }
//
//        // Parse sender
//        if (!getline(ss, token, ',')) continue;
//        int attackerId;
//        try {
//            attackerId = std::stoi(token);
//        } catch (const std::exception&) {
//            continue;  // Skip invalid lines
//        }
//
//        // Parse attackType
//        if (!getline(ss, token, ',')) continue;
//        int aType;
//
//        // Map attack type string to enum
//        if (token == "DoS" || token == "dos" || token == "DOS") {
//            aType = ATTACK_TYPE_DOS;
//        } else {
//            try {
//                aType = std::stoi(token);
//                // Only accept DoS attacks (type 1)
//                if (aType != ATTACK_TYPE_DOS) {
//                    continue;  // Skip unsupported attack types
//                }
//            } catch (const std::exception&) {
//                continue;  // Skip invalid lines
//            }
//        }
//
//        // Only add attack events for this car
//        if (attackerId == mySenderId) {
//            AttackEvent event;
//            event.time = attackTime;
//            event.attackType = aType;
//            attackSchedule.push_back(event);
//        }
//    }
//
//    // Sort attack schedule by time
//    std::sort(attackSchedule.begin(), attackSchedule.end(),
//        [](const AttackEvent& a, const AttackEvent& b) {
//            return a.time < b.time;
//        });
//
//    EV_INFO << "Loaded " << attackSchedule.size() << " DoS attack events for car ID " << mySenderId << endl;
//}
//
//void CarApp::checkForAttack()
//{
//    // Current simulation time
//    double now = simTime().dbl();
//
//    // Check for attack start or end
//    for (const auto& event : attackSchedule) {
//        if (event.time <= now && !isAttacking) {
//            // Only start DoS attacks, ignore other attack types
//            if (event.attackType == ATTACK_TYPE_DOS) {
//                startAttack(event.attackType);
//            }
//            break;
//        }
//        else if (isAttacking && event.time > now) {
//            // End current attack
//            stopAttack();
//            break;
//        }
//    }
//}
//
//void CarApp::startAttack(int attackType)
//{
//    if (!isAttacking) {
//        isAttacking = true;
//        currentAttackType = attackType;
//
//        // Only DoS attacks are supported now
//        if (currentAttackType == ATTACK_TYPE_DOS) {
//            EV_INFO << "Starting DoS attack at " << simTime() << endl;
//        } else {
//            // Reset attack status for unsupported attack types
//            isAttacking = false;
//            currentAttackType = ATTACK_TYPE_NONE;
//            EV_INFO << "Attack type " << attackType << " not supported, ignoring at " << simTime() << endl;
//        }
//    }
//}
//
//void CarApp::stopAttack()
//{
//    if (isAttacking) {
//        EV_INFO << "Stopping attack at " << simTime() << endl;
//        isAttacking = false;
//        currentAttackType = ATTACK_TYPE_NONE;
//    }
//}
//
//void CarApp::sendNormalMessage()
//{
//    // Create a new packet
//    std::string packetName = "Car_" + std::to_string(mySenderId) + "_" + std::to_string(seqNumber++);
//    auto packet = new Packet(packetName.c_str());
//
//    // Create payload with vehicle data
//    auto payload = makeShared<BytesChunk>();
//    std::vector<uint8_t> data(100, 0);  // 100 byte packet
//
//    // Include sender ID and message type
//    data[0] = mySenderId & 0xFF;
//    data[1] = (mySenderId >> 8) & 0xFF;
//    data[2] = 0x00;  // Normal message type
//
//    // Include position data
//    uint32_t xInt = static_cast<uint32_t>(currentX * 100.0);  // Centimeter precision
//    uint32_t yInt = static_cast<uint32_t>(currentY * 100.0);
//
//    data[3] = (xInt >> 0) & 0xFF;
//    data[4] = (xInt >> 8) & 0xFF;
//    data[5] = (xInt >> 16) & 0xFF;
//    data[6] = (xInt >> 24) & 0xFF;
//
//    data[7] = (yInt >> 0) & 0xFF;
//    data[8] = (yInt >> 8) & 0xFF;
//    data[9] = (yInt >> 16) & 0xFF;
//    data[10] = (yInt >> 24) & 0xFF;
//
//    // Include speed and direction
//    uint16_t speedInt = static_cast<uint16_t>(currentSpeed * 100.0);  // Centimeter/s precision
//    uint16_t dirInt = static_cast<uint16_t>((currentDirection + M_PI) * 1000.0); // Milliradian precision
//
//    data[11] = (speedInt >> 0) & 0xFF;
//    data[12] = (speedInt >> 8) & 0xFF;
//
//    data[13] = (dirInt >> 0) & 0xFF;
//    data[14] = (dirInt >> 8) & 0xFF;
//
//    // Set the data in the payload
//    payload->setBytes(data);
//    packet->insertAtBack(payload);
//
//    // Add timestamp tag
//    packet->addTag<CreationTimeTag>()->setCreationTime(simTime());
//
//    // Add protocol tag (needed by the message dispatcher)
//    packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);
//
//    // Log before sending
//    EV_WARN << "Preparing to send normal message: " << packet->getName() << endl;
//
//    // Send the packet
//    sendToNic(packet);
//
//    // Emit signal for metrics collection
//    emit(packetSentSignal, 1.0);
//    getParentModule()->emit(packetSentSignal, 1.0);  // Also emit at parent level for better propagation
//
//    // Update statistics
//    packetSentVector.record(1);
//}
//
//void CarApp::sendDoSAttack()
//{
//    // DoS attack sends multiple packets in a short time
//    int numPackets = par("dosPacketCount");
//
//    for (int i = 0; i < numPackets; i++) {
//        // Create a packet similar to normal but with attack marker
//        std::string packetName = "Car_" + std::to_string(mySenderId) + "_DoS_" + std::to_string(seqNumber++);
//        auto packet = new Packet(packetName.c_str());
//
//        // Create larger payload to consume bandwidth
//        auto payload = makeShared<BytesChunk>();
//        std::vector<uint8_t> data(500, 0);  // 500 byte packet (5x normal)
//
//        // Include sender ID and message type
//        data[0] = mySenderId & 0xFF;
//        data[1] = (mySenderId >> 8) & 0xFF;
//        data[2] = 0x01;  // DoS attack marker
//
//        // Set the data in the payload
//        payload->setBytes(data);
//        packet->insertAtBack(payload);
//
//        // Add timestamp tag
//        packet->addTag<CreationTimeTag>()->setCreationTime(simTime());
//
//        // Add protocol tag (needed by the message dispatcher)
//        packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);
//
//        // Send the packet
//        sendToNic(packet);
//    }
//
//    // Emit total packets sent in this DoS attack
//    emit(packetSentSignal, (double)numPackets);
//    getParentModule()->emit(packetSentSignal, (double)numPackets);
//
//    // Update statistics
//    attackRateVector.record(numPackets);
//}
//void CarApp::processReceivedPacket(Packet *packet)
//{
//    // Process incoming packets (e.g., from other vehicles)
//    EV_WARN << "PACKET RECEIVED by Car[" << mySenderId << "]: " << packet->getName() << " at time " << simTime() << endl;
//
//    // Extract data from the packet
//    auto payload = packet->peekData<BytesChunk>();
//    if (payload) {
//        // Process the data
//        auto& bytes = payload->getBytes();
//
//        if (bytes.size() >= 3) {
//            // Extract sender ID
//            int senderId = bytes[0] | (bytes[1] << 8);
//
//            // Extract message type
//            int messageType = bytes[2];
//
//            // Extract position if available
//            double posX = 0.0, posY = 0.0;
//            if (bytes.size() >= 11) {
//                uint32_t xInt = bytes[3] | (bytes[4] << 8) | (bytes[5] << 16) | (bytes[6] << 24);
//                uint32_t yInt = bytes[7] | (bytes[8] << 8) | (bytes[9] << 16) | (bytes[10] << 24);
//
//                posX = xInt / 100.0;  // Convert from centimeters to meters
//                posY = yInt / 100.0;
//            }
//
//            // Extract speed and direction if available
//            double speed = 0.0, direction = 0.0;
//            if (bytes.size() >= 15) {
//                uint16_t speedInt = bytes[11] | (bytes[12] << 8);
//                uint16_t dirInt = bytes[13] | (bytes[14] << 8);
//
//                speed = speedInt / 100.0;     // Convert from cm/s to m/s
//                direction = (dirInt / 1000.0) - M_PI;  // Convert from milliradians and shift range
//            }
//
//            // Process based on message type
//            std::string msgTypeStr;
//            switch (messageType) {
//                case 0x00:
//                    msgTypeStr = "Normal";
//                    break;
//                case 0x01:
//                    msgTypeStr = "DoS";
//                    break;
//                default:
//                    msgTypeStr = "Unknown";
//                    break;
//            }
//
//            EV_INFO << "  - Sender ID: " << senderId << endl;
//            EV_INFO << "  - Message Type: " << msgTypeStr << endl;
//            EV_INFO << "  - Position: (" << posX << ", " << posY << ")" << endl;
//            EV_INFO << "  - Speed: " << speed << " m/s, Direction: " << direction << " rad" << endl;
//
//            // Emit that a packet was successfully received
//            emit(packetReceivedSignal, 1.0);
//            getParentModule()->emit(packetReceivedSignal, 1.0);  // Also emit at parent level
//
//            // Calculate and emit end-to-end delay if timestamp is available
//            auto timeTag = packet->findTag<CreationTimeTag>();
//            if (timeTag) {
//                simtime_t delay = simTime() - timeTag->getCreationTime();
//                EV_INFO << "  - End-to-end delay: " << delay << " s" << endl;
//                emit(endToEndDelaySignal, delay.dbl());
//            }
//        }
//    }
//}
//
//void CarApp::initializeRDO()
//{
//    EV_INFO << "car[" << mySenderId << "] initializing RDO" << endl;
//
//    // Check if forceTrusted parameter exists
//    try {
//        forceTrusted = par("forceTrusted").boolValue();
//    } catch (const std::exception& e) {
//        // Parameter doesn't exist, use default (false)
//        forceTrusted = false;
//    }
//
//    // Determine trust status with multiple options
//    // 1. gNBs with ID >= 1000 are trusted
//    // 2. Nodes with the forceTrusted parameter
//    // 3. Nodes with IDs 0, 1, 2 (to match gNBs in ini file)
//    isTrusted = (mySenderId >= 1000) || forceTrusted || (mySenderId == 0 || mySenderId == 1 || mySenderId == 2);
//
//    EV_WARN << "Node " << mySenderId << " trust status: " << (isTrusted ? "TRUSTED" : "NOT TRUSTED") << endl;
//
//    // Initialize cost function parameters - consistent with both implementations
//    costFunctionTarget = 50.0 + mySenderId * 10.0;
//    costFunctionWeight = 1.0 + (mySenderId % 3) * 0.2;
//
//    // Define trusted gNB IDs (from original implementation)
//    trustedGnbIds = {1000, 1001, 1002};
//
//    // Also add some cars as trusted (for initial trust setup)
//    for (int i = 0; i < 4; i++) {
//        if (std::find(trustedGnbIds.begin(), trustedGnbIds.end(), i) == trustedGnbIds.end()) {
//            trustedGnbIds.push_back(i);
//        }
//    }
//
//    // Initialize tracking metrics
//    lastReceivedTime = 0.0;
//    totalRdoPacketsReceived = 0;
//    totalRdoPacketsSent = 0;
//
//    // Create RDO agent with sender ID and trust status
//    rdoAgent = new RDOAgent(mySenderId, isTrusted);
//
//    // Set filter type from configuration (combining both approaches)
//    // Using the loaded configuration if available, otherwise default to type 3
//    int filterType = (loadedFilterType > 0) ? loadedFilterType : 3;
//    rdoAgent->setFilterType(filterType);
//
//    // Set consensus algorithm from configuration
//    int consensusAlgo = (loadedConsensusAlgorithm > 0) ? loadedConsensusAlgorithm : 1;
//    rdoAgent->setConsensusAlgorithm(consensusAlgo);
//
//    // Set cost function target and max state change in agent
//    rdoAgent->setCostFunctionTarget(costFunctionTarget);
//    rdoAgent->setMaxStateChange(10.0);  // Limit state changes for stability
//
//    // Build neighbor list (all cars + trusted gNBs)
//    std::vector<int> allNeighbors;
//    int numCars = getSystemModule()->par("numCars").intValue();
//
//    // Add all cars except self
//    for (int i = 0; i < numCars; i++) {
//        if (i != mySenderId) {
//            allNeighbors.push_back(i);
//        }
//    }
//
//    // Add all trusted gNBs
//    for (int gnbId : trustedGnbIds) {
//        if (gnbId >= 1000) {  // Only add actual gNBs here
//            allNeighbors.push_back(gnbId);  // Keep original IDs (1000+)
//        }
//    }
//
//    // Initialize RDO agent state using position as initial state
//    // IMPORTANT FIX: Ensure non-zero initial state
//    double initialState = currentX;
//
//    // Check if initial state is zero or too small
//    if (std::abs(initialState) < 0.1) {
//        // Force a non-zero initial state based on node ID
//        initialState = 10.0 + mySenderId * 5.0;
//        std::cout << "FORCING NON-ZERO INITIAL STATE " << initialState
//                  << " for node " << mySenderId << std::endl;
//    }
//
//    rdoAgent->initialize(initialState, trustedGnbIds, allNeighbors);
//
//    // Calculate initial gradient and set it
//    double gradient = generateGradient();
//    rdoAgent->setGradient(gradient);
//
//    // Record initial state and gradient
//    rdoStateVector.record(initialState);
//    rdoGradientVector.record(gradient);
//
//    // Only trusted nodes broadcast their states initially
//    if (isTrusted) {
//        sendRDOState();
//    }
//
//    EV_INFO << "Initialized RDO agent for node " << mySenderId
//            << " with filterType=" << filterType
//            << ", consensusAlgo=" << consensusAlgo
//            << ", initialState=" << initialState
//            << ", isTrusted=" << isTrusted
//            << ", neighbors=" << allNeighbors.size() << endl;
//
//    // Direct console output for more visibility
//    std::cout << "Initialized RDO agent for node " << mySenderId
//              << " with initialState=" << initialState
//              << ", target=" << costFunctionTarget
//              << ", gradient=" << gradient << std::endl;
//}
//
//void CarApp::processRDOUpdate()
//{
//    if (!rdoAgent) {
//        EV_ERROR << "RDO agent is not initialized in processRDOUpdate" << endl;
//        return;
//    }
//
//    // Log the current state of neighborStates
//    EV_WARN << "Car[" << mySenderId << "] updating RDO state at time " << simTime()
//            << ". neighborStates size: " << neighborStates.size()
//            << ", total packets received: " << totalRdoPacketsReceived
//            << ", total packets sent: " << totalRdoPacketsSent << endl;
//
//    // Generate gradient based on local cost function
//    double gradient = generateGradient();
//    rdoAgent->setGradient(gradient);
//
//    // Update step size according to the paper: αk decreases with iterations
//    double iterationNumber = simTime().dbl() / 0.5;
//
//    // IMPORTANT FIX: Use a larger step size initially
//    double stepSize;
//    if (iterationNumber < 5.0) {
//        // Use larger step size for first few iterations
//        stepSize = 0.5;
//    } else {
//        // Then switch to decreasing step size
//        stepSize = 1.0 / (iterationNumber + 1.0);
//    }
//
//    std::cout << "Step size for node " << mySenderId << " at time " << simTime().dbl()
//              << " (iteration " << iterationNumber << "): " << stepSize << std::endl;
//
//    // Get current state for comparison
//    double oldState = rdoAgent->getState();
//
//    // Update state with projection (from original implementation)
//    // This combines both implementations' approaches
//    double newState = rdoAgent->updateStateWithProjection(neighborStates, stepSize);
//    rdoAgent->setState(newState);
//
//    // Record updated state and gradient values
//    rdoStateVector.record(newState);
//    rdoGradientVector.record(gradient);
//
//    // Calculate and record consensus value
//    std::map<int, double> allStates = neighborStates;
//    allStates[mySenderId] = newState;
//
//    double consensusValue = 0.0;
//    if (!allStates.empty()) {
//        for (auto& entry : allStates) {
//            consensusValue += entry.second;
//            EV_DEBUG << "  Including in consensus: Node " << entry.first
//                    << " state=" << entry.second << endl;
//        }
//        consensusValue /= allStates.size();
//
//        EV_INFO << "Calculated consensus value: " << consensusValue
//                << " based on " << allStates.size() << " states" << endl;
//
//        rdoConsensusVector.record(consensusValue);
//    } else {
//        // If no neighbor states available, use own state
//        consensusValue = newState;
//        EV_WARN << "No neighbor states available - using own state for consensus: "
//                << consensusValue << endl;
//        rdoConsensusVector.record(consensusValue);
//    }
//
//    // Calculate the value of the local cost function
//    double costValue = costFunctionWeight * pow(newState - costFunctionTarget, 2);
//
//    EV_INFO << "Updated state for node " << mySenderId
//            << " from " << oldState << " to " << newState
//            << " (change: " << (newState - oldState) << ")"
//            << " (step size=" << stepSize
//            << ", cost value=" << costValue << ")" << endl;
//
//    // Direct console output for visibility
//    std::cout << "RDO Update for node " << mySenderId
//              << " at time " << simTime().dbl()
//              << ": " << oldState << " -> " << newState
//              << " (change: " << (newState - oldState) << ")"
//              << ", target=" << costFunctionTarget
//              << ", gap=" << std::abs(newState - costFunctionTarget) << std::endl;
//
//    // Only trusted nodes broadcast their states
//    if (isTrusted) {
//        sendRDOState();
//    }
//}
//
//void CarApp::sendRDOState()
//{
//    if (!rdoAgent) return;
//
//    // Create packet name with sequence number
//    std::string packetName = "RDOState_" + std::to_string(mySenderId) + "_" + std::to_string(seqNumber++);
//    auto packet = new Packet(packetName.c_str());
//
//    // Create payload chunk
//    auto payload = makeShared<BytesChunk>();
//    std::vector<uint8_t> data(100, 0);  // 100 byte packet
//
//    // Include sender ID and RDO message type
//    data[0] = mySenderId & 0xFF;
//    data[1] = (mySenderId >> 8) & 0xFF;
//    data[2] = 0x10;  // RDO state message type
//
//    // Include state value (double - 8 bytes)
//    double state = rdoAgent->getState();
//    uint64_t stateBits;
//    memcpy(&stateBits, &state, sizeof(double));
//    for (int i = 0; i < 8; i++) {
//        data[3 + i] = (stateBits >> (i * 8)) & 0xFF;
//    }
//
//    // Include gradient value (double - 8 bytes)
//    double grad = rdoAgent->getGradient();
//    uint64_t gradBits;
//    memcpy(&gradBits, &grad, sizeof(double));
//    for (int i = 0; i < 8; i++) {
//        data[11 + i] = (gradBits >> (i * 8)) & 0xFF;
//    }
//
//    // Set payload bytes
//    payload->setBytes(data);
//    packet->insertAtBack(payload);
//
//    // Add tags
//    packet->addTag<CreationTimeTag>()->setCreationTime(simTime());
//    packet->addTag<PacketProtocolTag>()->setProtocol(&Protocol::udp);
//
//    // Log packet sending
//    EV_WARN << "Car[" << mySenderId << "] sending RDO state packet #" << totalRdoPacketsSent
//            << " at time " << simTime() << ": "
//            << packetName << " with state=" << state
//            << ", gradient=" << grad << endl;
//
//    // Use broadcast for RDO state dissemination
//    try {
//        L3Address broadcastAddr = L3AddressResolver().resolve("255.255.255.255");
//        socket.sendTo(packet, broadcastAddr, 9000);
//        totalRdoPacketsSent++;
//
//        EV_WARN << "Broadcasting RDO state to " << broadcastAddr.str() << ":9000" << endl;
//    } catch (const std::exception& e) {
//        EV_ERROR << "Exception in sendRDOState: " << e.what() << endl;
//        delete packet;  // Prevent memory leak
//    }
//}
//
//void CarApp::processRDOStatePacket(Packet* packet)
//{
//    auto payload = packet->peekData<BytesChunk>();
//    if (!payload || payload->getBytes().size() < 19) {
//        EV_ERROR << "Invalid RDO state packet: insufficient data" << endl;
//        return;
//    }
//
//    auto& bytes = payload->getBytes();
//
//    // Extract sender ID
//    int senderId = bytes[0] | (bytes[1] << 8);
//
//    // Verify packet is from a trusted source by combining both trust models
//    bool isTrustedSender = false;
//
//    // Check if sender is in the trusted gNB list (IDs 1000+)
//    if (std::find(trustedGnbIds.begin(), trustedGnbIds.end(), senderId) != trustedGnbIds.end()) {
//        isTrustedSender = true;
//    }
//
//    // Alternative approach: trusted car IDs (from second implementation)
//    std::vector<int> trustedCarIds = {0, 1, 2};  // Trusted car IDs from expanded implementation
//    if (std::find(trustedCarIds.begin(), trustedCarIds.end(), senderId) != trustedCarIds.end()) {
//        isTrustedSender = true;
//    }
//
//    if (!isTrustedSender) {
//        EV_WARN << "Ignoring RDO state from untrusted node: " << senderId << endl;
//        return;
//    }
//
//    // Verify message type
//    if (bytes[2] != 0x10) {
//        EV_ERROR << "Expected RDO state message (0x10), but got " << static_cast<int>(bytes[2]) << endl;
//        return;
//    }
//
//    // Extract state value
//    uint64_t stateInt = 0;
//    for (int i = 0; i < 8; i++) {
//        stateInt |= static_cast<uint64_t>(bytes[3 + i]) << (i * 8);
//    }
//    double stateVal;
//    memcpy(&stateVal, &stateInt, sizeof(double));
//
//    // Extract gradient value (for logging)
//    uint64_t gradientInt = 0;
//    for (int i = 0; i < 8; i++) {
//        gradientInt |= static_cast<uint64_t>(bytes[11 + i]) << (i * 8);
//    }
//    double gradientVal;
//    memcpy(&gradientVal, &gradientInt, sizeof(double));
//
//    // Update neighbor state map
//    neighborStates[senderId] = stateVal;
//
//    // Update tracking metrics
//    lastReceivedTime = simTime().dbl();
//    totalRdoPacketsReceived++;
//
//    EV_WARN << "RDO state received from trusted node " << senderId
//            << ": state=" << stateVal
//            << ", gradient=" << gradientVal
//            << ", total packets received=" << totalRdoPacketsReceived << endl;
//}
//
//double CarApp::generateGradient()
//{
//    double state = rdoAgent ? rdoAgent->getState() : currentX;
//
//    // Using a quadratic cost function as described in the paper: fi(x) = wi(x - ai)²
//    // where ai is a target value specific to this node
//    // and wi is a weight parameter
//
//    // Compute the gradient of the cost function: f'i(x) = 2*wi*(x - ai)
//    double gradient = 2.0 * costFunctionWeight * (state - costFunctionTarget);
//
//    // Direct console output
//    std::cout << "Generated gradient for node " << mySenderId
//              << ": " << gradient
//              << " (state=" << state
//              << ", target=" << costFunctionTarget
//              << ", weight=" << costFunctionWeight << ")" << std::endl;
//
//    EV_INFO << "Generated gradient for node " << mySenderId
//            << ": " << gradient
//            << " (state=" << state
//            << ", target=" << costFunctionTarget
//            << ", weight=" << costFunctionWeight << ")" << endl;
//
//    return gradient;
//}
//
//int CarApp::getWlanInterfaceId()
//{
//    // Safe way to get the interface
//    if (!interfaceTable)
//        throw cRuntimeError("InterfaceTable not initialized properly!");
//
//    // First try to find cellularNic
//    NetworkInterface *entry = interfaceTable->findInterfaceByName("cellularNic");
//
//    // If not found, try alternative names
//    if (!entry) {
//        entry = interfaceTable->findInterfaceByName("wlan0");
//    }
//
//    // If still not found, try to get the first wireless interface
//    if (!entry) {
//        for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
//            NetworkInterface *iface = interfaceTable->getInterface(i);
//            if (iface && iface->isWireless()) {
//                entry = iface;
//                EV_INFO << "Using wireless interface: " << iface->getInterfaceName() << endl;
//                break;
//            }
//        }
//    }
//
//    // Last resort - use the first interface that's not loopback
//    if (!entry) {
//        for (int i = 0; i < interfaceTable->getNumInterfaces(); i++) {
//            NetworkInterface *iface = interfaceTable->getInterface(i);
//            if (iface && !iface->isLoopback()) {
//                entry = iface;
//                EV_WARN << "No wireless interface found, using: " << iface->getInterfaceName() << endl;
//                break;
//            }
//        }
//    }
//
//    if (!entry)
//        throw cRuntimeError("No suitable network interface found!");
//
//    return entry->getInterfaceId();
//}
//
//void CarApp::sendToNic(Packet *packet)
//{
//    try {
//        // Add debug info without using unavailable methods
//        EV_WARN << "SENDING PACKET: Car[" << mySenderId << "] "
//                << "packet: " << packet->getName()
//                << " size: " << packet->getByteLength() << " bytes" << endl;
//
//        // Just send using the UDP socket
//        socket.send(packet);
//
//        // Log success
//        EV_WARN << "Packet sent successfully" << endl;
//    } catch (const std::exception& e) {
//        EV_ERROR << "Exception in sendToNic: " << e.what() << endl;
//        delete packet;  // Prevent memory leak
//    }
//}








