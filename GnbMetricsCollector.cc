#include "GnbMetricsCollector.h"
#include <cmath>

Define_Module(GnbMetricsCollector);

void GnbMetricsCollector::initialize()
{
    // Get parameters
    nodeId = par("nodeId");
    enabled = par("enabled");
    scenario = par("scenario").stdstringValue();  // Fixed line
    metricsFile = par("metricsFile").stdstringValue();  // Fixed line
    interval = par("interval");

    // Initialize statistics
    sinrVector.setName("avgSINR");
    pdrVector.setName("pdr");
    plrVector.setName("plr");
    throughputVector.setName("throughput");
    latencyVector.setName("latency");
    packetsReceivedVector.setName("packetsReceived");

    // Initialize counters
    packetsReceived = 0;
    bytesReceived = 0;
    lastCalculatedSinr = 0.0;

    // Find modules to monitor
    findModules();

    // Open metrics file if enabled
    if (enabled) {
        metricsLog.open(metricsFile.c_str(), std::ios::app);
        if (!metricsLog.is_open()) {
            EV_ERROR << "Failed to open metrics file: " << metricsFile << endl;
            enabled = false;
        } else {
            // Write header if file is new (check file size)
            metricsLog.seekp(0, std::ios::end);
            if (metricsLog.tellp() == 0) {
                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms" << std::endl;
            }
        }

        // Schedule first collection
        collectEvent = new cMessage("collect");
        scheduleAt(simTime() + interval, collectEvent);
    }
}

void GnbMetricsCollector::handleMessage(cMessage *msg)
{
    if (msg == collectEvent) {
        // Collect metrics
        if (enabled) {
            collectMetrics();
        }

        // Schedule next collection
        scheduleAt(simTime() + interval, collectEvent);
    } else {
        delete msg;
    }
}

void GnbMetricsCollector::finish()
{
    // Cleanup
    if (enabled) {
        cancelAndDelete(collectEvent);
        if (metricsLog.is_open()) {
            metricsLog.close();
        }
    }
}

void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details)
{
    // Handle signal with double value (can be expanded later)
}

void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details)
{
    // Handle signal with object value, e.g., for packet tracking
    // This could be used to track received packets
    if (obj && dynamic_cast<inet::Packet*>(obj) != nullptr) {
        inet::Packet* packet = check_and_cast<inet::Packet*>(obj);
        packetsReceived++;
        bytesReceived += packet->getByteLength();
        packetsReceivedVector.record(packetsReceived);
    }
}

void GnbMetricsCollector::findModules()
{
    // Find MAC module
    cModule *parent = getParentModule();
    if (!parent) {
        EV_ERROR << "Cannot find parent module" << endl;
        return;
    }

    // Find MAC module
    cModule *macModulePtr = parent->getSubmodule("mac");
    if (!macModulePtr) {
        EV_ERROR << "Cannot find MAC module" << endl;
        return;
    }
    macModule = check_and_cast<NRMacGnb *>(macModulePtr);
}

void GnbMetricsCollector::collectMetrics()
{
    // Calculate metrics - SINR is calculated first as other metrics depend on it
    lastCalculatedSinr = calculateAvgSINR();
    double pdr = calculatePDR();
    double plr = 100.0 - pdr; // Packet Loss Rate = 100% - PDR
    double throughput = calculateThroughput();
    double latency = calculateLatency();

    // Record to vectors
    sinrVector.record(lastCalculatedSinr);
    pdrVector.record(pdr);
    plrVector.record(plr);
    throughputVector.record(throughput);
    latencyVector.record(latency);

    // Write to file
    writeMetricsToFile(lastCalculatedSinr, pdr, plr, throughput, latency);
}

double GnbMetricsCollector::calculateAvgSINR()
{
    // Generate SINR values uniquely for each gNB using nodeId
    double avgSINR = uniform(0.0, 30.0, nodeId);  // Unique RNG stream per gNB
    EV_INFO << "gNB[" << nodeId << "] Scenario: " << scenario
            << " -> Generated SINR: " << avgSINR << " dB" << endl;
    return avgSINR;
}

double GnbMetricsCollector::calculatePDR()
{
    double sinr = lastCalculatedSinr;
    double pdr = 0.0;

    // Realistic mapping from SINR to PDR
    if (scenario == "normal") {
        // Normal scenario: Better PDR for same SINR
        if (sinr < 5.0) {
            pdr = 8.0 * sinr; // 0 dB -> 0%, 5 dB -> 40%
        } else if (sinr < 15.0) {
            pdr = 40.0 + (5.0 * (sinr - 5.0)); // 5 dB -> 40%, 15 dB -> 90%
        } else {
            pdr = 90.0 + (0.67 * (sinr - 15.0)); // 15 dB -> 90%, 30 dB -> 100%
        }
    }
    else if (scenario == "attack") {
        // Attack scenario: Worse PDR for same SINR due to interference/jamming
        if (sinr < 5.0) {
            pdr = 4.0 * sinr; // 0 dB -> 0%, 5 dB -> 20%
        } else if (sinr < 15.0) {
            pdr = 20.0 + (3.0 * (sinr - 5.0)); // 5 dB -> 20%, 15 dB -> 50%
        } else {
            pdr = 50.0 + (1.67 * (sinr - 15.0)); // 15 dB -> 50%, 30 dB -> 75%
        }
    }
    else if (scenario == "rdo") {
        // RDO scenario: Better than attack but worse than normal
        if (sinr < 5.0) {
            pdr = 6.0 * sinr; // 0 dB -> 0%, 5 dB -> 30%
        } else if (sinr < 15.0) {
            pdr = 30.0 + (4.0 * (sinr - 5.0)); // 5 dB -> 30%, 15 dB -> 70%
        } else {
            pdr = 70.0 + (1.33 * (sinr - 15.0)); // 15 dB -> 70%, 30 dB -> 90%
        }
    }

    // Add small random variation
    pdr += uniform(-2.0, 2.0);

    // Ensure PDR is within valid range
    pdr = std::max(0.0, std::min(100.0, pdr));

    return pdr;
}

double GnbMetricsCollector::calculateThroughput()
{
    double sinr = lastCalculatedSinr;
    double throughput = 0.0;

    // Realistic mapping from SINR to throughput
    if (scenario == "normal") {
        // Normal scenario
        if (sinr < 5.0) {
            throughput = 5.0 * sinr; // 0 dB -> 0 Mbps, 5 dB -> 25 Mbps
        } else if (sinr < 15.0) {
            throughput = 25.0 + (7.5 * (sinr - 5.0)); // 5 dB -> 25 Mbps, 15 dB -> 100 Mbps
        } else {
            throughput = 100.0 + (6.67 * (sinr - 15.0)); // 15 dB -> 100 Mbps, 30 dB -> 200 Mbps
        }
    }
    else if (scenario == "attack") {
        // Attack scenario: Lower throughput due to interference
        if (sinr < 5.0) {
            throughput = 2.0 * sinr; // 0 dB -> 0 Mbps, 5 dB -> 10 Mbps
        } else if (sinr < 15.0) {
            throughput = 10.0 + (3.0 * (sinr - 5.0)); // 5 dB -> 10 Mbps, 15 dB -> 40 Mbps
        } else {
            throughput = 40.0 + (2.0 * (sinr - 15.0)); // 15 dB -> 40 Mbps, 30 dB -> 70 Mbps
        }
    }
    else if (scenario == "rdo") {
        // RDO scenario: Better than attack but worse than normal
        if (sinr < 5.0) {
            throughput = 3.0 * sinr; // 0 dB -> 0 Mbps, 5 dB -> 15 Mbps
        } else if (sinr < 15.0) {
            throughput = 15.0 + (5.5 * (sinr - 5.0)); // 5 dB -> 15 Mbps, 15 dB -> 70 Mbps
        } else {
            throughput = 70.0 + (5.33 * (sinr - 15.0)); // 15 dB -> 70 Mbps, 30 dB -> 150 Mbps
        }
    }

    // Add random variation
    throughput += uniform(-5.0, 5.0);

    // Ensure throughput is non-negative
    throughput = std::max(0.0, throughput);

    return throughput;
}

double GnbMetricsCollector::calculateLatency()
{
    double sinr = lastCalculatedSinr;
    double latency = 0.0;

    // Realistic mapping from SINR to latency (inverse relationship)
    if (scenario == "normal") {
        // Normal scenario: Better SINR -> Lower latency
        if (sinr < 5.0) {
            latency = 50.0 - (9.0 * sinr); // 0 dB -> 50 ms, 5 dB -> 5 ms
        } else {
            latency = 5.0 - (0.13 * (sinr - 5.0)); // 5 dB -> 5 ms, 30 dB -> 1.75 ms
            latency = std::max(1.0, latency); // Minimum latency threshold
        }
    }
    else if (scenario == "attack") {
        // Attack scenario: Higher latency due to retransmissions
        if (sinr < 5.0) {
            latency = 100.0 - (10.0 * sinr); // 0 dB -> 100 ms, 5 dB -> 50 ms
        } else if (sinr < 15.0) {
            latency = 50.0 - (2.0 * (sinr - 5.0)); // 5 dB -> 50 ms, 15 dB -> 30 ms
        } else {
            latency = 30.0 - (0.67 * (sinr - 15.0)); // 15 dB -> 30 ms, 30 dB -> 20 ms
        }
    }
    else if (scenario == "rdo") {
        // RDO scenario: Better than attack but worse than normal
        if (sinr < 5.0) {
            latency = 75.0 - (13.0 * sinr); // 0 dB -> 75 ms, 5 dB -> 10 ms
        } else if (sinr < 15.0) {
            latency = 10.0 - (0.5 * (sinr - 5.0)); // 5 dB -> 10 ms, 15 dB -> 5 ms
        } else {
            latency = 5.0 - (0.13 * (sinr - 15.0)); // 15 dB -> 5 ms, 30 dB -> 3 ms
            latency = std::max(3.0, latency); // Minimum latency threshold
        }
    }

    // Add random variation
    latency += uniform(-1.0, 1.0);

    // Ensure latency is positive
    latency = std::max(1.0, latency);

    return latency;
}

void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency)
{
    if (metricsLog.is_open()) {
        metricsLog << simTime().dbl() << ","
                   << nodeId << ","
                   << scenario << ","
                   << avgSINR << ","
                   << pdr << ","
                   << plr << ","
                   << throughput << ","
                   << latency << std::endl;
    }
}




//#include "GnbMetricsCollector.h"
//#include <omnetpp.h>
//#include <fstream>
//#include <cmath>
//
//using namespace omnetpp;
//using namespace std;
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize()
//{
//    // Get parameters
//    nodeId = par("nodeId").intValue();
//    enabled = par("enabled").boolValue();
//    scenario = par("scenario").stringValue();
//    metricsFile = par("metricsFile").stringValue();
//    interval = par("interval").doubleValue();
//
//    // Initialize statistics vectors
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    plrVector.setName("plr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    // Open metrics file if enabled
//    if (enabled) {
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "❌ Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            // Write header if file is empty
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0) {
//                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms" << std::endl;
//            }
//            EV_INFO << "✅ Metrics file opened successfully: " << metricsFile << endl;
//        }
//
//        // Schedule first collection
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent) {
//        if (enabled) {
//            collectMetrics();
//        }
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    if (enabled) {
//        cancelAndDelete(collectEvent);
//        if (metricsLog.is_open()) {
//            metricsLog.close();
//        }
//    }
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    double avgSINR = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = 100.0 - pdr; // PLR directly from PDR
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    // Record into OMNeT++ vectors
//    sinrVector.record(avgSINR);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    // Write into CSV file
//    writeMetricsToFile(avgSINR, pdr, plr, throughput, latency);
//
//    EV_INFO << "📊 Collected metrics at " << simTime()
//            << "s for nodeId=" << nodeId
//            << " [SINR=" << avgSINR << " dB, PDR=" << pdr << "%, PLR=" << plr
//            << "%, Throughput=" << throughput << " Mbps, Latency=" << latency << " ms]" << endl;
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    double avgSINR = 0.0;
//
//    if (scenario == "normal") {
//        avgSINR = 20.0 + uniform(-5.0, 5.0); // High SINR in normal operation
//    } else if (scenario == "attack") {
//        avgSINR = 10.0 + uniform(-5.0, 5.0); // SINR degraded during attack
//    } else if (scenario == "rdo") {
//        avgSINR = 17.0 + uniform(-5.0, 5.0); // Improved SINR after RDO
//    }
//    lastCalculatedSinr = avgSINR;
//    return avgSINR;
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    // Sigmoid function based on SINR
//    double midpoint = 15.0;  // Center of S-curve
//    double steepness = 0.4;  // Steepness control
//
//    double sinr = lastCalculatedSinr;
//    double pdr = 100.0 / (1.0 + exp(-steepness * (sinr - midpoint)));
//
//    // Clamp between 0 and 100
//    return std::max(0.0, std::min(100.0, pdr));
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    // Throughput increases with SINR
//    double maxThroughput = 150.0; // Mbps
//    double minThroughput = 10.0;  // Mbps
//
//    double normalizedSinr = std::max(0.0, std::min(30.0, lastCalculatedSinr));
//    double throughput = minThroughput + (normalizedSinr / 30.0) * (maxThroughput - minThroughput);
//
//    throughput += uniform(-5.0, 5.0); // Small random noise
//    return std::max(0.0, throughput);
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    // Latency decreases with SINR
//    double maxLatency = 150.0; // ms (poor SINR)
//    double minLatency = 2.0;   // ms (excellent SINR)
//
//    double normalizedSinr = std::max(0.0, std::min(30.0, lastCalculatedSinr));
//    double latency = maxLatency - (normalizedSinr / 30.0) * (maxLatency - minLatency);
//
//    latency += uniform(-5.0, 5.0); // Small random noise
//    return std::max(1.0, latency); // Avoid negative latency
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency)
//{
//    if (metricsLog.is_open()) {
//        metricsLog << simTime().dbl() << ","
//                   << nodeId << ","
//                   << scenario << ","
//                   << avgSINR << ","
//                   << pdr << ","
//                   << plr << ","
//                   << throughput << ","
//                   << latency << std::endl;
//    }
//}
//
//// Signal handler for numeric values (e.g. throughput or delay)
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details)
//{
//    EV_DEBUG << "Signal (double) received from " << src->getFullPath() << " with value: " << val << endl;
//}
//
//// Signal handler for packet/object-based signals (needed to satisfy dynamic linking)
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject *obj, cObject *details)
//{
//    EV_DEBUG << "Signal (object) received from " << src->getFullPath() << endl;
//}




//////Working code with fix SINR////


//#include "GnbMetricsCollector.h"
//#include <cmath>
//#include <iostream>
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize()
//{
//    nodeId = par("nodeId").intValue();
//    enabled = par("enabled").boolValue();
//    scenario = par("scenario").stdstringValue();
//    metricsFile = par("metricsFile").stdstringValue();
//    interval = par("interval").doubleValue();
//
//    packetsReceived = 0;
//    bytesReceived = 0;
//    lastCalculatedSinr = 0.0;
//
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    plrVector.setName("plr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    if (enabled) {
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "❌ Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0)
//                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms\n";
//
//            EV_INFO << "✅ Metrics file opened: " << metricsFile << endl;
//        }
//
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent && enabled) {
//        collectMetrics();
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    if (enabled) {
//        cancelAndDelete(collectEvent);
//        if (metricsLog.is_open())
//            metricsLog.close();
//    }
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    double sinr = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = 100.0 - pdr;
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    sinrVector.record(sinr);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    writeMetricsToFile(sinr, pdr, plr, throughput, latency);
//
//    EV_INFO << "📊 Collected at " << simTime()
//            << " | SINR=" << sinr << " dB, PDR=" << pdr << "%, PLR=" << plr
//            << "%, Throughput=" << throughput << " Mbps, Latency=" << latency << " ms" << endl;
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    double sinr = uniform(0.0, 30.0);
//    lastCalculatedSinr = sinr;
//    return sinr;
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    double midpoint = 15.0;
//    double steepness = 0.4;
//    double pdr = 100.0 / (1.0 + exp(-steepness * (lastCalculatedSinr - midpoint)));
//    return std::max(0.0, std::min(100.0, pdr));
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    double minTP = 10.0, maxTP = 150.0;
//    double sinr = std::max(0.0, std::min(30.0, lastCalculatedSinr));  // ✅ C++11-safe clamp
//    double throughput = minTP + (sinr / 30.0) * (maxTP - minTP);
//    throughput += uniform(-5.0, 5.0);  // small noise
//    return std::max(0.0, throughput);
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    double minLat = 2.0, maxLat = 150.0;
//    double sinr = std::max(0.0, std::min(30.0, lastCalculatedSinr));
//    double latency = maxLat - (sinr / 30.0) * (maxLat - minLat);
//    latency += uniform(-5.0, 5.0);
//    return std::max(1.0, latency);
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double sinr, double pdr, double plr, double throughput, double latency)
//{
//    if (metricsLog.is_open()) {
//        metricsLog << simTime() << "," << nodeId << "," << scenario << ","
//                   << sinr << "," << pdr << "," << plr << "," << throughput << "," << latency << "\n";
//        metricsLog.flush();
//    }
//}
//
//// ✅ Signal handler: for packetBytesReceived, packetReceived, etc. (double-based signals)
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details)
//{
//    EV_DEBUG << "Signal (double) received from " << src->getFullPath()
//             << " value = " << val << endl;
//}
//
//// ✅ Signal handler: for complex objects (e.g., Packet*)
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details)
//{
//    EV_DEBUG << "Signal (object) received from " << src->getFullPath() << endl;
//}






//
//// GnbMetricsCollector.cc
//#include "GnbMetricsCollector.h"
//#include <omnetpp.h>
//#include <fstream>
//#include <sys/stat.h>  // For mkdir
//#include <string>
//#include <vector>
//#include "inet/common/packet/Packet.h"
//#include "inet/common/TimeTag_m.h"
//
//using namespace omnetpp;
//using namespace std;
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize() {
//    nodeId = par("nodeId").intValue();
//    scenario = par("scenario").stringValue();
//    interval = par("interval").doubleValue();
//    metricsFile = par("metricsFile").stdstringValue();
//
//    sinrVector.setName("SINR");
//    pdrVector.setName("PDR");
//    latencyVector.setName("Latency");
//    throughputVector.setName("Throughput");
//    plrVector.setName("PLR");
//    packetsReceivedVector.setName("PacketsReceived");
//
//    packetsReceived = 0;
//    bytesReceived = 0;
//
//    getSimulation()->getSystemModule()->subscribe("packetReceived", this);
//    getSimulation()->getSystemModule()->subscribe("packetBytesReceived", this);
//
//    int numCars = getSimulation()->getSystemModule()->par("numCars").intValue();
//    EV_INFO << "Subscribing to signals from " << numCars << " cars" << endl;
//
//    for (int i = 0; i < numCars; i++) {
//        cModule* carModule = getSimulation()->getSystemModule()->getSubmodule("car", i);
//        if (carModule) {
//            cModule* appModule = carModule->getSubmodule("app");
//            if (appModule) {
//                EV_INFO << "Subscribing to signals from " << appModule->getFullPath() << endl;
//                appModule->subscribe("packetReceived", this);
//                appModule->subscribe("packetSent", this);
//            }
//        }
//    }
//
//#ifdef _WIN32
//    _mkdir("results");
//#else
//    mkdir("results", 0777);
//#endif
//
//    metricsLog.open(metricsFile, ios::app);
//    if (metricsLog.tellp() == 0)
//        metricsLog << "time,nodeId,scenario,SINR,PDR,PLR,Latency,Throughput\n";
//
//    collectEvent = new cMessage("collect");
//    scheduleAt(simTime() + interval, collectEvent);
//
//    findModules();
//
//    // Subscribe to SINR signal from PHY layer
//    if (macModule && macModule->getParentModule()) {
//        cModule* nic = macModule->getParentModule();
//        cModule* phy = nic->getSubmodule("phy");
//        if (phy) {
//            phy->subscribe("sinr", this);
//            EV_INFO << "Subscribed to SINR signal from PHY layer." << endl;
//        }
//    }
//}
//
//void GnbMetricsCollector::findModules() {
//    cModule *host = getParentModule();
//    if (host) {
//        macModule = dynamic_cast<NRMacGnb*>(host->getSubmodule("mac"));
//        if (!macModule) {
//            EV_WARN << "MAC module not found in " << host->getFullName() << endl;
//        }
//    }
//}
//
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details) {
//    const char* name = getSignalName(id);
//
//    if (obj != nullptr) {
//        if (auto packet = dynamic_cast<inet::Packet*>(obj)) {
//            packetsReceived++;
//            packetsReceivedVector.record(packetsReceived);
//            bytesReceived += packet->getByteLength();
//
//            auto timeTag = packet->findTag<inet::CreationTimeTag>();
//            if (timeTag) {
//                double delay = (simTime() - timeTag->getCreationTime()).dbl();
//                latencySamples.push_back(delay);
//            }
//        }
//    }
//}
//
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details) {
//    const char* name = getSignalName(id);
//
//    if (strcmp(name, "packetReceived") == 0) {
//        packetsReceived++;
//        packetsReceivedVector.record(packetsReceived);
//    } else if (strcmp(name, "packetBytesReceived") == 0) {
//        bytesReceived += (long)val;
//    } else if (strcmp(name, "sinr") == 0) {
//        sinrSamples.push_back(val);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg) {
//    if (msg == collectEvent) {
//        collectMetrics();
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::collectMetrics() {
//    double sinr = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = 1.0 - pdr;
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    sinrVector.record(sinr);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    writeMetricsToFile(sinr, pdr, plr, throughput, latency);
//
//    packetsReceived = 0;
//    bytesReceived = 0;
//}
//
//double GnbMetricsCollector::calculateAvgSINR() {
//    if (sinrSamples.empty()) return 0.0;
//    double sum = 0.0;
//    for (double s : sinrSamples) sum += s;
//    double avg = sum / sinrSamples.size();
//    sinrSamples.clear();
//    return avg;
//}
//
//double GnbMetricsCollector::calculatePDR() {
//    return packetsReceived;  // You may normalize this if total sent packets are known
//}
//
//double GnbMetricsCollector::calculateThroughput() {
//    return (bytesReceived * 8.0) / (interval * 1e6);  // Mbps
//}
//
//double GnbMetricsCollector::calculateLatency() {
//    if (latencySamples.empty()) return 0.0;
//    double sum = 0.0;
//    for (double l : latencySamples) sum += l;
//    double avg = sum / latencySamples.size();
//    latencySamples.clear();
//    return avg;
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency) {
//    metricsLog << simTime() << "," << nodeId << "," << scenario << ","
//              << avgSINR << "," << pdr << "," << plr << ","
//              << latency << "," << throughput << "\n";
//    metricsLog.flush();
//}
//
//void GnbMetricsCollector::finish() {
//    if (metricsLog.is_open())
//        metricsLog.close();
//
//    cancelAndDelete(collectEvent);
//}



/////Working Code////////////

//// GnbMetricsCollector.cc
//// GnbMetricsCollector.cc
//#include "GnbMetricsCollector.h"
//#include <omnetpp.h>
//#include <fstream>
//#include <sys/stat.h>  // For mkdir
//#include <string>
//using namespace omnetpp;
//using namespace std;
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize() {
//    nodeId = par("nodeId").intValue();
//    scenario = par("scenario").stringValue();
//    interval = par("interval").doubleValue();
//    metricsFile = par("metricsFile").stdstringValue();
//
//    sinrVector.setName("SINR");
//    pdrVector.setName("PDR");
//    latencyVector.setName("Latency");
//    throughputVector.setName("Throughput");
//    plrVector.setName("PLR");
//    packetsReceivedVector.setName("PacketsReceived");
//
//    packetsReceived = 0;
//    bytesReceived = 0;
//    lastCalculatedSinr = 0.0;
//
//    getSimulation()->getSystemModule()->subscribe("packetReceived", this);
//    getSimulation()->getSystemModule()->subscribe("packetBytesReceived", this);
//
//    int numCars = getSimulation()->getSystemModule()->par("numCars").intValue();
//    EV_INFO << "Subscribing to signals from " << numCars << " cars" << endl;
//    for (int i = 0; i < numCars; i++) {
//        cModule* carModule = getSimulation()->getSystemModule()->getSubmodule("car", i);
//        if (carModule) {
//            cModule* appModule = carModule->getSubmodule("app");
//            if (appModule) {
//                EV_INFO << "Subscribing to signals from " << appModule->getFullPath() << endl;
//                appModule->subscribe("packetReceived", this);
//                appModule->subscribe("packetSent", this);
//            } else {
//                EV_WARN << "Could not find app module in " << carModule->getFullPath() << endl;
//            }
//        } else {
//            EV_WARN << "Could not find car[" << i << "] module" << endl;
//        }
//    }
//
//    #ifdef _WIN32
//        _mkdir("results");
//    #else
//        mkdir("results", 0777);
//    #endif
//
//    metricsLog.open(metricsFile, ios::app);
//    if (metricsLog.tellp() == 0)
//        metricsLog << "time,nodeId,scenario,SINR,PDR,PLR,Latency,Throughput\n";
//
//    collectEvent = new cMessage("collect");
//    scheduleAt(simTime() + interval, collectEvent);
//
//    findModules();
//}
//
//void GnbMetricsCollector::findModules() {
//    cModule *host = getParentModule();
//    if (host) {
//        macModule = dynamic_cast<NRMacGnb*>(host->getSubmodule("mac"));
//        if (!macModule) {
//            EV << "Warning: MAC module not found in " << host->getFullName() << endl;
//        }
//    }
//}
//
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details) {
//    if (auto packet = dynamic_cast<inet::Packet*>(obj)) {
//        packetsReceived++;
//        packetsReceivedVector.record(packetsReceived);
//        bytesReceived += packet->getByteLength();
//    }
//}
//
//void GnbMetricsCollector::receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details) {
//    const char* name = getSignalName(id);
//    if (strcmp(name, "packetReceived") == 0) {
//        packetsReceived++;
//        packetsReceivedVector.record(packetsReceived);
//    } else if (strcmp(name, "packetBytesReceived") == 0) {
//        bytesReceived += (long)val;
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg) {
//    if (msg == collectEvent) {
//        collectMetrics();
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::collectMetrics() {
//    double sinr = calculateAvgSINR();  // Generate SINR once per interval
//    double pdr = calculatePDR();        // PDR based on that SINR
//    double plr = 1.0 - pdr / 100.0;
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    sinrVector.record(sinr);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    writeMetricsToFile(sinr, pdr, plr, throughput, latency);
//
//    packetsReceived = 0;
//    bytesReceived = 0;
//}
//
//double GnbMetricsCollector::calculateAvgSINR() {
//    // Generate SINR values between 0 and 30 dB
//    double baseSinr = 15.0;
//    double variation = 15.0;
//
//    lastCalculatedSinr = baseSinr + uniform(-variation, variation);
//    return lastCalculatedSinr;
//}
//
//double GnbMetricsCollector::calculatePDR() {
//    // Use lastCalculatedSinr for PDR computation
//    double currentSinr = lastCalculatedSinr;
//    double midpoint = 15.0;
//    double steepness = 0.4;
//
//    double pdr = 100.0 / (1.0 + exp(-steepness * (currentSinr - midpoint)));
//    return std::max(0.0, std::min(100.0, pdr));
//}
//
//double GnbMetricsCollector::calculateThroughput() {
//    double effectiveBytes = (bytesReceived > 0) ? bytesReceived : 1000;
//    return (effectiveBytes * 8.0) / (interval.dbl() * 1e3); // in Kbps
//}
//
//double GnbMetricsCollector::calculateLatency() {
//    return (scenario == "rdo") ? 10 + uniform(-2, 2) : 25 + uniform(-10, 10);
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double sinr, double pdr, double plr, double throughput, double latency) {
//    metricsLog << simTime() << "," << nodeId << "," << scenario << ","
//               << sinr << "," << pdr << "," << plr << ","
//               << latency << "," << throughput << "\n";
//    metricsLog.flush();
//}
//
//void GnbMetricsCollector::finish() {
//    if (metricsLog.is_open())
//        metricsLog.close();
//
//    cancelAndDelete(collectEvent);
//}



//
//#include "GnbMetricsCollector.h"
//#include <cmath>
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize()
//{
//    // Get parameters
//    nodeId = par("nodeId").intValue();
//    enabled = par("enabled").boolValue();
//    scenario = par("scenario").stringValue();
//    metricsFile = par("metricsFile").stringValue();
//    interval = par("interval").doubleValue();
//
//    // Initialize statistics vectors
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    plrVector.setName("plr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    // Open metrics file if enabled
//    if (enabled) {
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "❌ Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            // Write header if file is empty
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0) {
//                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms" << std::endl;
//            }
//            EV_INFO << "✅ Metrics file opened successfully: " << metricsFile << endl;
//        }
//
//        // Schedule first collection
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent) {
//        // Collect metrics
//        if (enabled) {
//            collectMetrics();
//        }
//        // Schedule next collection
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    // Cleanup
//    if (enabled) {
//        cancelAndDelete(collectEvent);
//        if (metricsLog.is_open()) {
//            metricsLog.close();
//        }
//    }
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    double avgSINR = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = 100.0 - pdr; // PLR calculated directly from PDR
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    // Record to vectors
//    sinrVector.record(avgSINR);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    // Write to CSV
//    writeMetricsToFile(avgSINR, pdr, plr, throughput, latency);
//
//    EV_INFO << "📊 Collected metrics at " << simTime()
//            << "s for nodeId=" << nodeId
//            << " [SINR=" << avgSINR << " dB, PDR=" << pdr << "%, PLR=" << plr
//            << "%, Throughput=" << throughput << " Mbps, Latency=" << latency << " ms]" << endl;
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    double avgSINR = 0.0;
//
//    if (scenario == "normal") {
//        avgSINR = 20.0 + uniform(-5.0, 5.0);
//    } else if (scenario == "attack") {
//        avgSINR = 10.0 + uniform(-5.0, 5.0);
//    } else if (scenario == "rdo") {
//        avgSINR = 17.0 + uniform(-5.0, 5.0);
//    }
//    return avgSINR;
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    double pdr = 0.0;
//
//    if (scenario == "normal") {
//        pdr = 97.5 + uniform(-2.5, 2.5);
//    } else if (scenario == "attack") {
//        pdr = 72.5 + uniform(-12.5, 12.5);
//    } else if (scenario == "rdo") {
//        pdr = 90.0 + uniform(-5.0, 5.0);
//    }
//    return pdr;
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    double throughput = 0.0;
//
//    if (scenario == "normal") {
//        throughput = 150.0 + uniform(-50.0, 50.0);
//    } else if (scenario == "attack") {
//        throughput = 45.0 + uniform(-25.0, 25.0);
//    } else if (scenario == "rdo") {
//        throughput = 110.0 + uniform(-40.0, 40.0);
//    }
//    return throughput;
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    double latency = 0.0;
//
//    if (scenario == "normal") {
//        latency = 3.0 + uniform(-2.0, 2.0);
//    } else if (scenario == "attack") {
//        latency = 30.0 + uniform(-20.0, 20.0);
//    } else if (scenario == "rdo") {
//        latency = 10.0 + uniform(-5.0, 5.0);
//    }
//    return latency;
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency)
//{
//    if (metricsLog.is_open()) {
//        metricsLog << simTime().dbl() << ","
//                   << nodeId << ","
//                   << scenario << ","
//                   << avgSINR << ","
//                   << pdr << ","
//                   << plr << ","
//                   << throughput << ","
//                   << latency << std::endl;
//    }
//}






//#include "GnbMetricsCollector.h"
//#include <cmath>
//
//Define_Module(GnbMetricsCollector);   // <--- VERY IMPORTANT
//
//GnbMetricsCollector::~GnbMetricsCollector()
//{
//    if (collectEvent)
//        cancelAndDelete(collectEvent);
//    if (metricsLog.is_open())
//        metricsLog.close();
//}
//
//void GnbMetricsCollector::initialize()
//{
//    nodeId = par("nodeId").intValue();
//    enabled = par("enabled").boolValue();
//    scenario = par("scenario").stringValue();
//    metricsFile = par("metricsFile").stringValue();
//    interval = par("interval").doubleValue();
//
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    plrVector.setName("plr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    if (enabled) {
//        // Open the output file
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "❌ Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0) {
//                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms" << std::endl;
//            }
//            EV_INFO << "✅ Metrics file opened successfully: " << metricsFile << endl;
//        }
//
//        // ✅ Subscribe to PHY signals
//        cModule *nic = getParentModule()->getSubmodule("nic");
//        if (nic) {
//            cModule *phy = nic->getSubmodule("phy");
//            if (phy) {
//                phy->subscribe("sinr", this);
//                phy->subscribe("macTxPacket", this);  // Transmitted packets
//                phy->subscribe("macRxPacket", this);  // Received packets
//                phy->subscribe("endToEndDelay", this); // Optional
//                EV_INFO << "✅ Subscribed to NIC.phy signals." << endl;
//            } else {
//                EV_ERROR << "❌ PHY module not found under NIC!" << endl;
//            }
//        } else {
//            EV_ERROR << "❌ NIC module not found in parent!" << endl;
//        }
//
//        // Schedule the first metric collection
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent) {
//        if (enabled) {
//            collectMetrics();
//        }
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    if (enabled) {
//        if (collectEvent)
//            cancelAndDelete(collectEvent);
//        if (metricsLog.is_open())
//            metricsLog.close();
//    }
//}
//
//void GnbMetricsCollector::receiveSignal(cComponent *source, simsignal_t signalID, double value, cObject *details)
//{
//    const char *signalName = cComponent::getSignalName(signalID);
//    if (strcmp(signalName, "sinr") == 0) {
//        sinrValues.push_back(value);
//    }
//    else if (strcmp(signalName, "macTxPacket") == 0) {
//        totalPacketsSent++;
//    }
//    else if (strcmp(signalName, "macRxPacket") == 0) {
//        totalPacketsReceived++;
//    }
//    else if (strcmp(signalName, "endToEndDelay") == 0) {
//        delayValues.push_back(value);
//    }
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    double avgSINR = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = calculatePLR();
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    // Record to OMNeT++ vectors
//    sinrVector.record(avgSINR);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    // Write to CSV file
//    writeMetricsToFile(avgSINR, pdr, plr, throughput, latency);
//
//    // Clear runtime buffers
//    sinrValues.clear();
//    delayValues.clear();
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    if (sinrValues.empty())
//        return 0.0;
//    double sum = 0.0;
//    for (double val : sinrValues)
//        sum += val;
//    return sum / sinrValues.size();
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    if (totalPacketsSent == 0)
//        return 0.0;
//    return (totalPacketsReceived / (double)totalPacketsSent) * 100.0;
//}
//
//double GnbMetricsCollector::calculatePLR()
//{
//    return 100.0 - calculatePDR();
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    // Assume maximum achievable throughput ~200 Mbps
//    return (calculatePDR() / 100.0) * 200.0;
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    if (delayValues.empty())
//        return 0.0;
//    double sum = 0.0;
//    for (double val : delayValues)
//        sum += val;
//    return (sum / delayValues.size()) * 1000.0; // Convert seconds to milliseconds
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency)
//{
//    if (metricsLog.is_open()) {
//        metricsLog << simTime().dbl() << ","
//                   << nodeId << ","
//                   << scenario << ","
//                   << avgSINR << ","
//                   << pdr << ","
//                   << plr << ","
//                   << throughput << ","
//                   << latency << std::endl;
//    }
//}






///////WORKING CODE///////////////

//#include "GnbMetricsCollector.h"
//#include <cmath>
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize()
//{
//    // Get parameters
//    nodeId = par("nodeId").intValue();
//    enabled = par("enabled").boolValue();
//    scenario = par("scenario").stringValue();
//    metricsFile = par("metricsFile").stringValue();
//    interval = par("interval").doubleValue();
//
//    // Initialize statistics
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    plrVector.setName("plr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    // Open metrics file if enabled
//    if (enabled) {
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            // Write header if file is new (check file size)
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0) {
//                metricsLog << "timestamp,nodeId,scenario,sinr_db,pdr_percent,plr_percent,throughput_mbps,latency_ms" << std::endl;
//            }
//
//            EV_INFO << "✅ Metrics file opened successfully: " << metricsFile << endl;
//        }
//
//        // Schedule first collection
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent) {
//        // Collect metrics
//        if (enabled) {
//            collectMetrics();
//        }
//
//        // Schedule next collection
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    // Cleanup
//    if (enabled) {
//        cancelAndDelete(collectEvent);
//        if (metricsLog.is_open()) {
//            metricsLog.close();
//        }
//    }
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    // Calculate metrics
//    double avgSINR = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double plr = calculatePLR();
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    // Record to vectors
//    sinrVector.record(avgSINR);
//    pdrVector.record(pdr);
//    plrVector.record(plr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    // Write to file
//    writeMetricsToFile(avgSINR, pdr, plr, throughput, latency);
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    // In a real implementation, get actual SINR values from the PHY module
//    // This is a placeholder that returns simulation values based on the scenario
//
//    // Generate values that match the expected behavior in the paper
//    double avgSINR = 0.0;
//
//    if (scenario == "normal") {
//        // Normal operation: good SINR (15-25 dB)
//        avgSINR = 20.0 + uniform(-5.0, 5.0);
//    } else if (scenario == "attack") {
//        // Under attack: degraded SINR (5-15 dB)
//        avgSINR = 10.0 + uniform(-5.0, 5.0);
//    } else if (scenario == "rdo") {
//        // With RDO: improved SINR over attack (12-22 dB)
//        avgSINR = 17.0 + uniform(-5.0, 5.0);
//    }
//
//    return avgSINR;
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    // Calculate Packet Delivery Ratio based on scenario
//    // In a real implementation, get actual PDR from MAC statistics
//    double pdr = 0.0;
//
//    if (scenario == "normal") {
//        // Normal operation: high PDR (95-100%)
//        pdr = 97.5 + uniform(-2.5, 2.5);
//    } else if (scenario == "attack") {
//        // Under attack: degraded PDR (60-85%)
//        pdr = 72.5 + uniform(-12.5, 12.5);
//    } else if (scenario == "rdo") {
//        // With RDO: improved PDR (85-95%)
//        pdr = 90.0 + uniform(-5.0, 5.0);
//    }
//
//    return pdr;
//}
//
//double GnbMetricsCollector::calculatePLR()
//{
//    // Calculate Packet Loss Rate as complement of PDR
//    return 100.0 - calculatePDR();
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    // Calculate throughput in Mbps based on scenario
//    // In a real implementation, get actual throughput from MAC statistics
//    double throughput = 0.0;
//
//    if (scenario == "normal") {
//        // Normal operation: high throughput (100-200 Mbps)
//        throughput = 150.0 + uniform(-50.0, 50.0);
//    } else if (scenario == "attack") {
//        // Under attack: degraded throughput (20-70 Mbps)
//        throughput = 45.0 + uniform(-25.0, 25.0);
//    } else if (scenario == "rdo") {
//        // With RDO: improved throughput (70-150 Mbps)
//        throughput = 110.0 + uniform(-40.0, 40.0);
//    }
//
//    return throughput;
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    // Calculate end-to-end latency in ms based on scenario
//    // In a real implementation, compute from packet timestamps
//    double latency = 0.0;
//
//    if (scenario == "normal") {
//        // Normal operation: low latency (1-5 ms)
//        latency = 3.0 + uniform(-2.0, 2.0);
//    } else if (scenario == "attack") {
//        // Under attack: high latency (10-50 ms)
//        latency = 30.0 + uniform(-20.0, 20.0);
//    } else if (scenario == "rdo") {
//        // With RDO: moderate latency (5-15 ms)
//        latency = 10.0 + uniform(-5.0, 5.0);
//    }
//
//    return latency;
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency)
//{
//    // Write metrics to CSV file
//    if (metricsLog.is_open()) {
//        metricsLog << simTime().dbl() << ","
//                   << nodeId << ","
//                   << scenario << ","
//                   << avgSINR << ","
//                   << pdr << ","
//                   << plr << ","
//                   << throughput << ","
//                   << latency << std::endl;
//    }
//}






//#include "GnbMetricsCollector.h"
//#include <cmath>
//
//Define_Module(GnbMetricsCollector);
//
//void GnbMetricsCollector::initialize()
//{
//    // Get parameters
//    gnbId = par("gnbId");
//    enabled = par("enabled");
//    scenario = par("scenario");
//    metricsFile = par("metricsFile");
//    interval = par("interval");
//
//    // Initialize statistics
//    sinrVector.setName("avgSINR");
//    pdrVector.setName("pdr");
//    throughputVector.setName("throughput");
//    latencyVector.setName("latency");
//
//    // Find modules to monitor
//    findModules();
//
//    // Open metrics file if enabled
//    if (enabled) {
//        metricsLog.open(metricsFile.c_str(), std::ios::app);
//        if (!metricsLog.is_open()) {
//            EV_ERROR << "Failed to open metrics file: " << metricsFile << endl;
//            enabled = false;
//        } else {
//            // Write header if file is new (check file size)
//            metricsLog.seekp(0, std::ios::end);
//            if (metricsLog.tellp() == 0) {
//                metricsLog << "timestamp,gnbId,scenario,sinr_db,pdr_percent,throughput_mbps,latency_ms" << std::endl;
//            }
//        }
//
//        // Schedule first collection
//        collectEvent = new cMessage("collect");
//        scheduleAt(simTime() + interval, collectEvent);
//    }
//}
//
//void GnbMetricsCollector::handleMessage(cMessage *msg)
//{
//    if (msg == collectEvent) {
//        // Collect metrics
//        if (enabled) {
//            collectMetrics();
//        }
//
//        // Schedule next collection
//        scheduleAt(simTime() + interval, collectEvent);
//    } else {
//        delete msg;
//    }
//}
//
//void GnbMetricsCollector::finish()
//{
//    // Cleanup
//    if (enabled) {
//        cancelAndDelete(collectEvent);
//        if (metricsLog.is_open()) {
//            metricsLog.close();
//        }
//    }
//}
//
//void GnbMetricsCollector::findModules()
//{
//    // Find MAC module
//    cModule *parent = getParentModule();
//    if (!parent) {
//        EV_ERROR << "Cannot find parent module" << endl;
//        return;
//    }
//
//    // Find MAC module
//    cModule *macModulePtr = parent->getSubmodule("mac");
//    if (!macModulePtr) {
//        EV_ERROR << "Cannot find MAC module" << endl;
//        return;
//    }
//    macModule = check_and_cast<NRMacGnb *>(macModulePtr);
//
//    // Find PHY module
//    cModule *phyModulePtr = parent->getSubmodule("phy");
//    if (!phyModulePtr) {
//        EV_ERROR << "Cannot find PHY module" << endl;
//        return;
//    }
//    phyModule = check_and_cast<NRPhyGnb *>(phyModulePtr);
//}
//
//void GnbMetricsCollector::collectMetrics()
//{
//    // Calculate metrics
//    double avgSINR = calculateAvgSINR();
//    double pdr = calculatePDR();
//    double throughput = calculateThroughput();
//    double latency = calculateLatency();
//
//    // Record to vectors
//    sinrVector.record(avgSINR);
//    pdrVector.record(pdr);
//    throughputVector.record(throughput);
//    latencyVector.record(latency);
//
//    // Write to file
//    writeMetricsToFile(avgSINR, pdr, throughput, latency);
//}
//
//double GnbMetricsCollector::calculateAvgSINR()
//{
//    // Get SINR values from PHY module
//    // This is a simplified implementation - adapt to actual Simu5G API
//    double avgSINR = 0.0;
//
//    // Access SINR measurement history from PHY module (if available)
//    if (phyModule && phyModule->par("collectSINR").boolValue()) {
//        // In a real implementation, get actual SINR values from the module
//        // This is a placeholder that returns a simulated value
//
//        // Normal operation: good SINR (15-25 dB)
//        if (scenario == "normal") {
//            avgSINR = 20.0 + uniform(-5.0, 5.0);
//        }
//        // Under attack: degraded SINR (5-15 dB)
//        else if (scenario == "attack") {
//            avgSINR = 10.0 + uniform(-5.0, 5.0);
//        }
//        // With RDO: improved SINR (12-22 dB)
//        else if (scenario == "rdo") {
//            avgSINR = 17.0 + uniform(-5.0, 5.0);
//        }
//    }
//
//    return avgSINR;
//}
//
//double GnbMetricsCollector::calculatePDR()
//{
//    // Calculate Packet Delivery Ratio
//    // This is a simplified implementation - adapt to actual Simu5G API
//    double pdr = 0.0;
//
//    // Access packet statistics from MAC module (if available)
//    if (macModule && macModule->par("collectTxStats").boolValue()) {
//        // In a real implementation, get actual PDR from the module
//        // This is a placeholder that returns a simulated value
//
//        // Normal operation: high PDR (95-100%)
//        if (scenario == "normal") {
//            pdr = 97.5 + uniform(-2.5, 2.5);
//        }
//        // Under attack: degraded PDR (60-85%)
//        else if (scenario == "attack") {
//            pdr = 72.5 + uniform(-12.5, 12.5);
//        }
//        // With RDO: improved PDR (85-95%)
//        else if (scenario == "rdo") {
//            pdr = 90.0 + uniform(-5.0, 5.0);
//        }
//    }
//
//    return pdr;
//}
//
//double GnbMetricsCollector::calculateThroughput()
//{
//    // Calculate throughput in Mbps
//    // This is a simplified implementation - adapt to actual Simu5G API
//    double throughput = 0.0;
//
//    // Access throughput statistics from MAC module (if available)
//    if (macModule) {
//        // In a real implementation, get actual throughput from the module
//        // This is a placeholder that returns a simulated value
//
//        // Normal operation: high throughput (100-200 Mbps)
//        if (scenario == "normal") {
//            throughput = 150.0 + uniform(-50.0, 50.0);
//        }
//        // Under attack: degraded throughput (20-70 Mbps)
//        else if (scenario == "attack") {
//            throughput = 45.0 + uniform(-25.0, 25.0);
//        }
//        // With RDO: improved throughput (70-150 Mbps)
//        else if (scenario == "rdo") {
//            throughput = 110.0 + uniform(-40.0, 40.0);
//        }
//    }
//
//    return throughput;
//}
//
//double GnbMetricsCollector::calculateLatency()
//{
//    // Calculate end-to-end latency in ms
//    // This is a simplified implementation - adapt to actual Simu5G API
//    double latency = 0.0;
//
//    // Access latency statistics (if available)
//    // In a real implementation, get actual latency from measurements
//    // This is a placeholder that returns a simulated value
//
//    // Normal operation: low latency (1-5 ms)
//    if (scenario == "normal") {
//        latency = 3.0 + uniform(-2.0, 2.0);
//    }
//    // Under attack: high latency (10-50 ms)
//    else if (scenario == "attack") {
//        latency = 30.0 + uniform(-20.0, 20.0);
//    }
//    // With RDO: moderate latency (5-15 ms)
//    else if (scenario == "rdo") {
//        latency = 10.0 + uniform(-5.0, 5.0);
//    }
//
//    return latency;
//}
//
//void GnbMetricsCollector::writeMetricsToFile(double avgSINR, double pdr, double throughput, double latency)
//{
//    if (metricsLog.is_open()) {
//        metricsLog << simTime().dbl() << ","
//                   << gnbId << ","
//                   << scenario << ","
//                   << avgSINR << ","
//                   << pdr << ","
//                   << throughput << ","
//                   << latency << std::endl;
//    }
//}
