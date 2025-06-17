#ifndef __RDOAGENT_H__
#define __RDOAGENT_H__
#include <vector>
#include <map>
#include <string>
class RDOAgent {
public:
    // Constructor & initialization
    RDOAgent(int id, bool trusted);
    void initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all);
    // Accessors
    double getState() const;
    void setState(double newState);
    double getGradient() const;
    void setGradient(double newGradient);
    // Configuration methods
    void setFilterType(int type);
    void setConsensusAlgorithm(int algo);
    void setMaxStateChange(double maxChange);
    void setCostFunctionTarget(double target);
    void setConsensusWeight(double weight);
    // Update methods (multiple approaches)
    double updateStateWithProjection(const std::map<int, double>& neighborStates, double stepSize);
    double updateState(const std::map<int, double>& neighborStates, double stepSize);
    double updateStateWithConsensusAndProjection(const std::map<int, double>& neighborStates, double stepSize);
    // Filtering
    std::map<int, double> filterOutliers(const std::map<int, double>& neighborStates);
    // Helper methods
    double calculateConsensusValue(const std::map<int, double>& filteredStates);
    // Gradient outlier detection
    bool isGradientOutlier(double gradientValue);
    // History tracking
    std::vector<std::pair<double, double>> getConvergenceHistory() const;
private:
    // Node identification
    int nodeId;
    bool isTrusted;
    // State variables
    double state;
    double gradient;
    // Configuration parameters
    int filterType;
    int consensusAlgorithm;
    double maxStateChange;
    double costFunctionTarget;
    double consensusWeight;
    // Neighbor tracking
    std::vector<int> trustedNeighbors;
    std::vector<int> allNeighbors;
    // History tracking vectors
    std::vector<double> stateHistory;
    std::vector<double> consensusHistory;
    std::vector<double> gradientHistory;
};
// Helper functions for logging and file operations
bool directoryExists(const std::string& path);
void createDirectory(const std::string& path);
void logConvergence(double time, int nodeId, double oldState, double newState, double gradient,
                   double consensus, double rMin, double rMax, double intervalWidth, double optimalGap);
#endif // __RDOAGENT_H__

//////Working Code//////


//
//#ifndef __RDOAGENT_H
//#define __RDOAGENT_H
//
//#include <vector>
//#include <map>
//#include <cmath>
//#include <algorithm>
//#include <iostream>
//
///**
// * RDOAgent - Resilient Distributed Optimization with Trust
// *
// * Implements an agent that performs state updates based on gradient descent and
// * consensus with neighbor nodes, while incorporating trust and outlier filtering.
// */
//class RDOAgent {
//private:
//    int nodeId;                  // ID of this node
//    int filterType = 1;          // 1=none, 2=MAD outlier removal, 3=trusted-only
//    int consensusAlgorithm = 1;  // 1=average, 2=median, 3=trimmed mean
//    bool isTrusted;              // True if this node is a trusted agent (e.g., gNB)
//
//    double state = 0.0;          // Current internal state
//    double gradient = 0.0;       // Current gradient value
//
//    std::vector<int> trustedNeighbors;  // IDs of trusted neighbors
//    std::vector<int> allNeighbors;      // IDs of all neighbors
//
//    // Constants
//    double trustThreshold = 0.7;        // Threshold for trust-based filtering
//    double maxStateChange = 10.0;       // Stability constraint
//
//    // Filters neighbor state inputs based on outlier or trust criteria
//    std::map<int, double> filterOutliers(const std::map<int, double>& neighborStates);
//
//    // Utility to compute distance between state values
//    double stateDistance(double s1, double s2) const { return std::abs(s1 - s2); }
//
//public:
//    // Constructor
//    RDOAgent(int id, bool trusted);
//
//    // Initialization
//    void initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all);
//
//    // Accessors
//    double getState() const;
//    void setState(double newState);
//    double getGradient() const;
//    void setGradient(double newGradient);
//
//    // Algorithm Configuration
//    void setFilterType(int type);
//    void setConsensusAlgorithm(int algo);
//
//    // Main update step
//    double updateState(const std::map<int, double>& neighborStates, double stepSize);
//};
//
//#endif // __RDOAGENT_H





//#ifndef __RDOAGENT_H
//#define __RDOAGENT_H
//
//#include <vector>
//#include <map>
//#include <cmath>
//#include <algorithm>
//#include <iostream>
//
///**
// * RDOAgent - Resilient Distributed Optimization with Trust
// *
// * This class implements an RDO-T agent for distributed consensus in vehicular networks
// * with protection against malicious nodes. The algorithm gives higher weight to trusted
// * nodes (in this case, the gNBs) while still considering input from all nodes.
// */
//class RDOAgent {
//private:
//    int nodeId;             // ID of this node
//    int filterType = 1; // 1=none, 2=outlier, 3=trust_based
//    int consensusAlgorithm = 1; // 1=average, 2=median, 3=trimmed_mean
//    bool isTrusted;         // Whether this node is trusted (gNBs are trusted)
//    double state;           // Current state value
//    double gradient;        // Current gradient value
//
//    std::vector<int> trustedNeighbors;  // IDs of trusted neighbors (gNBs)
//    std::vector<int> allNeighbors;      // IDs of all neighbors
//
//    // Trust parameters
//    double trustThreshold = 0.7;        // Minimum trust score to consider a node
//    double maxStateChange = 10.0;       // Maximum allowed state change per iteration
//
//    // Filter outliers from neighbor states
//    std::map<int, double> filterOutliers(const std::map<int, double>& neighborStates);
//
//    // Distance function for state values
//    double stateDistance(double s1, double s2) { return std::abs(s1 - s2); }
//
//public:
//    RDOAgent(int id, bool trusted) : nodeId(id), isTrusted(trusted), state(0.0), gradient(0.0) {}
//
//    // Initialize with initial state and neighbor information
//    void initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all) {
//        state = initialState;
//        trustedNeighbors = trusted;
//        allNeighbors = all;
//    }
//
//    // Get current state
//    double getState() const { return state; }
//
//    // Set current state
//    void setState(double newState) { state = newState; }
//
//    // Get current gradient
//    double getGradient() const { return gradient; }
//
//    // Set current gradient
//    void setGradient(double newGradient) { gradient = newGradient; }
//
//    // Update state using RDO-T algorithm
//    double updateState(const std::map<int, double>& neighborStates, double stepSize);
//
//    void setFilterType(int type) { filterType = type; }
//    void setConsensusAlgorithm(int algo) { consensusAlgorithm = algo; }
//
//};
//
//#endif // __RDOAGENT_H
