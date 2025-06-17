#ifndef __GNBMETRICSCOLLECTOR_H_
#define __GNBMETRICSCOLLECTOR_H_

#include <omnetpp.h>
#include <fstream>
#include "inet/common/packet/Packet.h"
#include "stack/mac/layer/NRMacGnb.h"  // Update path if needed for your NRMacGnb

using namespace omnetpp;

class GnbMetricsCollector : public cSimpleModule, public cListener
{
private:
    // Parameters
    int nodeId;
    bool enabled;                     // ✅ Required: allows .cc to compile
    std::string scenario;
    simtime_t interval;
    std::string metricsFile;

    // Statistics vectors
    cOutVector sinrVector;
    cOutVector pdrVector;
    cOutVector latencyVector;
    cOutVector throughputVector;
    cOutVector plrVector;
    cOutVector packetsReceivedVector;

    // Internal counters
    int packetsReceived;
    long bytesReceived;
    double lastCalculatedSinr;       // Current SINR used for all calculations

    // Output file stream
    std::ofstream metricsLog;

    // Self-message for periodic execution
    cMessage *collectEvent;

    // Optional: reference to MAC layer
    NRMacGnb *macModule;

    // Internal methods
    void findModules();
    void collectMetrics();
    double calculateAvgSINR();
    double calculatePDR();
    double calculateThroughput();
    double calculateLatency();
    void writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency);

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    // Signal handlers (can be empty or used for packet logging later)
    virtual void receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details) override;
    virtual void receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details) override;
};

#endif





//#ifndef __GNBMETRICSCOLLECTOR_H_
//#define __GNBMETRICSCOLLECTOR_H_
//
//#include <omnetpp.h>
//#include <fstream>
//#include "inet/common/packet/Packet.h"
//#include "stack/mac/layer/NRMacGnb.h" // Include your MAC module header
//
//using namespace omnetpp;
//
//class GnbMetricsCollector : public cSimpleModule, public cListener
//{
//private:
//    // Parameters
//    int nodeId;
//    std::string scenario;
//    simtime_t interval;
//    std::string metricsFile;
//
//    // Statistics
//    cOutVector sinrVector;
//    cOutVector pdrVector;
//    cOutVector latencyVector;
//    cOutVector throughputVector;
//    cOutVector plrVector;
//    cOutVector packetsReceivedVector;
//
//    // Counters
//    int packetsReceived;
//    long bytesReceived;
//    double lastCalculatedSinr; // For storing the most recent SINR value
//
//    // File output
//    std::ofstream metricsLog;
//
//    // Self-message for periodic collection
//    cMessage *collectEvent;
//
//    // Reference to MAC module for additional metrics
//    NRMacGnb *macModule;
//
//    // Helper methods
//    void findModules();
//    void collectMetrics();
//    double calculateAvgSINR();
//    double calculatePDR();
//    double calculateThroughput();
//    double calculateLatency();
//    void writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency);
//
//protected:
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//
//    // Signal handlers
//    virtual void receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details) override;
//    virtual void receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details) override;
//};
//
//#endif



//#ifndef GNBMETRICSCOLLECTOR_H_
//#define GNBMETRICSCOLLECTOR_H_
//#include <omnetpp.h>
//#include <fstream>
//#include <string>
//#include "stack/mac/layer/NRMacGnb.h"
//using namespace omnetpp;
///**
// * Metrics collector module for gNBs
// * Collects SINR, PDR, throughput, latency, and PLR metrics
// */
//class GnbMetricsCollector : public cSimpleModule, public cListener
//{
//  private:
//    // Parameters
//    int nodeId;
//    bool enabled;
//    std::string scenario;
//    std::string metricsFile;
//    // Operational
//    cMessage *collectEvent;
//    simtime_t interval;
//    std::ofstream metricsLog;
//    // References
//    NRMacGnb *macModule;
//    // Stats
//    cOutVector sinrVector;
//    cOutVector pdrVector;
//    cOutVector plrVector;
//    cOutVector throughputVector;
//    cOutVector latencyVector;
//    cOutVector packetsReceivedVector;
//    int packetsReceived;
//    long bytesReceived;  // Added this missing variable
//  protected:
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//    virtual void receiveSignal(cComponent *src, simsignal_t id, double val, cObject *details) override;
//    // Add to the class declaration in GnbMetricsCollector.h
//    virtual void receiveSignal(cComponent *src, simsignal_t id, cObject* obj, cObject *details) override;
//    void findModules();
//    void collectMetrics();
//    double calculateAvgSINR();
//    double calculatePDR();
//    double calculateThroughput();
//    double calculateLatency();
//    void writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency);
//};
//#endif // GNBMETRICSCOLLECTOR_H_




//#ifndef _GNBMETRICSCOLLECTOR_H_
//#define _GNBMETRICSCOLLECTOR_H_
//
//#include <omnetpp.h>
//#include <fstream>
//#include <string>
//#include "stack/mac/layer/NRMacGnb.h"  // ✅ This is fine
//
//using namespace omnetpp;
//
///**
// * Metrics collector module for gNBs
// * Collects SINR, PDR, throughput, latency, and PLR metrics
// */
//class GnbMetricsCollector : public cSimpleModule
//{
//  private:
//    // Parameters
//    int nodeId;
//    bool enabled;
//    std::string scenario;   // Scenario type: "normal", "attack", or "rdo"
//    std::string metricsFile; // Output file path for metrics CSV
//
//    // Operational
//    cMessage *collectEvent;
//    simtime_t interval;
//    std::ofstream metricsLog;
//
//    // References to monitored modules
//    NRMacGnb *macModule; // Only MAC module
//    // No NRPhyGnb here!
//
//    // Statistics
//    cOutVector sinrVector;
//    cOutVector pdrVector;
//    cOutVector plrVector;
//    cOutVector throughputVector;
//    cOutVector latencyVector;
//
//  protected:
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//
//    // Helper methods
//    void findModules();
//    void collectMetrics();
//    double calculateAvgSINR();
//    double calculatePDR();
//    double calculateThroughput();
//    double calculateLatency();
//    void writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency);
//};
//
//#endif // _GNBMETRICSCOLLECTOR_H_






//#ifndef __RDOEXPERIMENT_GNBMETRICSCOLLECTOR_H_
//#define __RDOEXPERIMENT_GNBMETRICSCOLLECTOR_H_
//
//#include <omnetpp.h>
//#include <fstream>
//#include <string>
//#include <vector>
//
//using namespace omnetpp;
//
///**
// * GnbMetricsCollector
// * ---------------------
// * Module that collects and logs gNB performance metrics:
// * - SINR
// * - Packet Delivery Ratio (PDR)
// * - Packet Loss Ratio (PLR)
// * - Throughput
// * - Latency
// *
// * Metrics are written periodically to a CSV file.
// */
//class GnbMetricsCollector : public cSimpleModule, public cListener
//{
//  private:
//    // Configuration
//    int nodeId;               // ID of the gNB
//    bool enabled;             // Enable/disable metrics collection
//    std::string scenario;     // Scenario label: "normal", "attack", or "rdo"
//    std::string metricsFile;  // Path to save CSV file
//    double interval;          // Time interval for metric collection (seconds)
//
//    // Event scheduling
//    cMessage *collectEvent = nullptr;
//
//    // Output file
//    std::ofstream metricsLog;
//
//    // Runtime Buffers
//    std::vector<double> sinrValues;   // SINR measurements during interval
//    std::vector<double> delayValues;  // Delay measurements during interval
//    int totalPacketsSent = 0;
//    int totalPacketsReceived = 0;
//
//    // Vectors for OMNeT++ recording
//    cOutVector sinrVector;
//    cOutVector pdrVector;
//    cOutVector plrVector;
//    cOutVector throughputVector;
//    cOutVector latencyVector;
//
//  public:
//    virtual ~GnbMetricsCollector();
//
//  protected:
//    // OMNeT++ lifecycle
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//    virtual void receiveSignal(cComponent *source, simsignal_t signalID, double value, cObject *details) override;
//
//    // Metrics collection
//    void collectMetrics();
//    double calculateAvgSINR();
//    double calculatePDR();
//    double calculatePLR();
//    double calculateThroughput();
//    double calculateLatency();
//    void writeMetricsToFile(double avgSINR, double pdr, double plr, double throughput, double latency);
//};
//
//#endif // __RDOEXPERIMENT_GNBMETRICSCOLLECTOR_H_
//















//
//#ifndef _GNBMETRICSCOLLECTOR_H_
//#define _GNBMETRICSCOLLECTOR_H_
//
//#include <omnetpp.h>
//#include <fstream>
//#include <string>
//#include "stack/mac/layer/NRMacGnb.h"
//#include "stack/phy/layer/NRPhyGnb.h"
//
//using namespace omnetpp;
//
///**
// * Metrics collector module for gNBs
// * Collects SINR, PDR, throughput, and latency metrics
// */
//class GnbMetricsCollector : public cSimpleModule
//{
//  private:
//    // Parameters
//    int gnbId;
//    bool enabled;
//    std::string scenario; // "normal", "attack", or "rdo"
//    std::string metricsFile;
//
//    // Operational
//    cMessage *collectEvent;
//    simtime_t interval;
//    std::ofstream metricsLog;
//
//    // References to monitored modules
//    NRMacGnb *macModule;
//    NRPhyGnb *phyModule;
//
//    // Statistics
//    cOutVector sinrVector;
//    cOutVector pdrVector;
//    cOutVector throughputVector;
//    cOutVector latencyVector;
//
//  protected:
//    virtual void initialize() override;
//    virtual void handleMessage(cMessage *msg) override;
//    virtual void finish() override;
//
//    // Helper methods
//    void findModules();
//    void collectMetrics();
//    double calculateAvgSINR();
//    double calculatePDR();
//    double calculateThroughput();
//    double calculateLatency();
//    void writeMetricsToFile(double avgSINR, double pdr, double throughput, double latency);
//};
//
//#endif // _GNBMETRICSCOLLECTOR_H_
