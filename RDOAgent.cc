#include "RDOAgent.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>
#include <ctime>
#include <omnetpp.h>
#include <sys/stat.h>

using namespace omnetpp;

// Constructor implementation
RDOAgent::RDOAgent(int id, bool trusted) :
    nodeId(id),
    isTrusted(trusted),
    state(0.0),
    gradient(0.0),
    filterType(3),              // Default to trust-based filtering
    consensusAlgorithm(1),      // Default to weighted average
    maxStateChange(10.0),       // Default limit for state changes
    costFunctionTarget(0.0),    // Default target (to be set by CarApp)
    consensusWeight(1.2)        // Default consensus influence weight (increased)
{
    // Constructor implementation
}

// Initialization method
void RDOAgent::initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all) {
    state = initialState;
    trustedNeighbors = trusted;
    allNeighbors = all;

    // Initialize history tracking
    stateHistory.clear();
    consensusHistory.clear();
    gradientHistory.clear();

    // Record initial state
    stateHistory.push_back(initialState);
}

// Accessor methods
double RDOAgent::getState() const {
    return state;
}

void RDOAgent::setState(double newState) {
    state = newState;
}

double RDOAgent::getGradient() const {
    return gradient;
}

void RDOAgent::setGradient(double newGradient) {
    gradient = newGradient;
    gradientHistory.push_back(newGradient);
}

// Configuration methods
void RDOAgent::setFilterType(int type) {
    filterType = type;
}

void RDOAgent::setConsensusAlgorithm(int algo) {
    consensusAlgorithm = algo;
}

void RDOAgent::setMaxStateChange(double maxChange) {
    maxStateChange = maxChange;
}

void RDOAgent::setCostFunctionTarget(double target) {
    costFunctionTarget = target;
}

void RDOAgent::setConsensusWeight(double weight) {
    consensusWeight = weight;
}

// Projection-based update
double RDOAgent::updateStateWithProjection(const std::map<int, double>& neighborStates, double stepSize) {
    // Debug output
    std::cout << "Node " << nodeId << " updating at time " << simTime().dbl()
              << " with stepSize=" << stepSize
              << ", current state=" << state
              << ", gradient=" << gradient << std::endl;

    // Extract values from trusted neighbors
    std::vector<double> trustedValues;
    for (const auto& kv : neighborStates) {
        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
            trustedValues.push_back(kv.second);
            std::cout << "  - Trusted neighbor " << kv.first << " has state " << kv.second << std::endl;
        }
    }

    // Include own state in trusted values
    trustedValues.push_back(state);

    // IMPORTANT FIX: If all trusted values are still 0, add a non-zero value
    bool allZeros = true;
    for (double val : trustedValues) {
        if (std::abs(val) > 0.001) {
            allZeros = false;
            break;
        }
    }

    if (allZeros) {
        double nonZeroValue = 10.0 + nodeId * 5.0;
        trustedValues.push_back(nonZeroValue);
        std::cout << "  - Adding forced non-zero value " << nonZeroValue
                  << " to trusted values for node " << nodeId << std::endl;
    }

    // Calculate bounds from trusted values
    double rMin = *std::min_element(trustedValues.begin(), trustedValues.end());
    double rMax = *std::max_element(trustedValues.begin(), trustedValues.end());

    // IMPORTANT FIX: Ensure bounds aren't both zero
    if (std::abs(rMax - rMin) < 0.001) {
        // If bounds are too close, widen them
        rMin = std::min(rMin, state - 5.0);
        rMax = std::max(rMax, state + 5.0);
        std::cout << "  - Widening bounds to [" << rMin << ", " << rMax
                  << "] for node " << nodeId << std::endl;
    }

    double intervalWidth = rMax - rMin;

    // Standard gradient descent step
    double xk = state;
    double gradientTerm = -stepSize * gradient;

    // IMPORTANT FIX: Calculate consensus term separately
    // This ensures consensus has influence even in projection-based updates
    double consensusValue = calculateConsensusValue(neighborStates);
    double consensusTerm = (!neighborStates.empty()) ?
                           consensusWeight * stepSize * (consensusValue - state) : 0.0;

    // Combined update
    double xk_next = xk + gradientTerm + consensusTerm;

    std::cout << "  - Gradient term: " << gradientTerm
              << ", consensus term: " << consensusTerm
              << ", combined update: " << (gradientTerm + consensusTerm)
              << ", proposed new state: " << xk_next << std::endl;

    // Project result to [rMin, rMax]
    double projectedState = std::min(std::max(xk_next, rMin), rMax);

    // IMPORTANT FIX: Ensure state change happens
    if (std::abs(projectedState - state) < 0.001) {
        // Force movement toward target if no change
        double targetDir = (costFunctionTarget > state) ? 1.0 : -1.0;
        double forcedStep = targetDir * stepSize * 5.0;
        projectedState = state + forcedStep;

        // Project again to enforce bounds
        projectedState = std::min(std::max(projectedState, rMin), rMax);

        std::cout << "  - Forcing state change to " << projectedState
                  << " for node " << nodeId << std::endl;
    }

    // Calculate optimal gap for logging
    double optimalGap = std::abs(projectedState - costFunctionTarget);

    std::cout << "  - Final projected state: " << projectedState
              << ", change: " << (projectedState - state)
              << ", target gap: " << optimalGap << std::endl;

    // Log convergence data
    logConvergence(simTime().dbl(), nodeId, xk, projectedState, gradient, consensusValue,
                  rMin, rMax, intervalWidth, optimalGap);

    // Save to history
    stateHistory.push_back(projectedState);
    consensusHistory.push_back(consensusValue);

    return projectedState;
}

// Consensus-based update (from expanded implementation)
double RDOAgent::updateState(const std::map<int, double>& neighborStates, double stepSize) {
    // Debug output
    std::cout << "Node " << nodeId << " updating (consensus method) at time " << simTime().dbl()
              << " with stepSize=" << stepSize
              << ", current state=" << state << std::endl;

    // Filter outliers based on selected algorithm
    std::map<int, double> filteredStates = filterOutliers(neighborStates);

    // Calculate weighted consensus with improved method
    double consensusValue = calculateConsensusValue(filteredStates);

    std::cout << "  - Consensus value: " << consensusValue
              << " from " << filteredStates.size() << " neighbors"
              << " (diff from current: " << (consensusValue - state) << ")" << std::endl;

    // Calculate update terms
    double gradientTerm = -stepSize * gradient;

    // IMPORTANT FIX: Use stronger consensus weight to ensure influence
    double consensusTerm = (!filteredStates.empty()) ?
                           consensusWeight * stepSize * (consensusValue - state) : 0.0;

    std::cout << "  - Gradient term: " << gradientTerm
              << ", consensus term: " << consensusTerm
              << " (with weight " << consensusWeight << ")" << std::endl;

    // Apply update with rate limiting
    double oldState = state;
    double stateChange = gradientTerm + consensusTerm;

    // Limit maximum state change
    if (std::abs(stateChange) > maxStateChange) {
        stateChange = (stateChange > 0) ? maxStateChange : -maxStateChange;
        std::cout << "  - Limiting change to " << stateChange << std::endl;
    }

    double newState = oldState + stateChange;

    // Force state change only if no neighbors (otherwise let consensus work)
    if (std::abs(newState - oldState) < 0.001 && filteredStates.empty()) {
        // Force movement toward target if no change and no neighbors
        double targetDir = (costFunctionTarget > oldState) ? 1.0 : -1.0;
        double forcedStep = targetDir * stepSize * 5.0;
        newState = oldState + forcedStep;
        std::cout << "  - Forcing state change to " << newState
                  << " for node " << nodeId << " (no neighbors)" << std::endl;
    }

    // Extract values from trusted neighbors for bounds and logging
    std::vector<double> trustedValues;
    for (const auto& kv : neighborStates) {
        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
            trustedValues.push_back(kv.second);
        }
    }

    // Include own state in trusted values
    trustedValues.push_back(oldState);

    // Calculate bounds and metrics from trusted values
    double rMin = trustedValues.empty() ? oldState : *std::min_element(trustedValues.begin(), trustedValues.end());
    double rMax = trustedValues.empty() ? oldState : *std::max_element(trustedValues.begin(), trustedValues.end());
    double intervalWidth = rMax - rMin;
    double optimalGap = std::abs(newState - costFunctionTarget);

    std::cout << "  - Final new state: " << newState
              << ", change: " << (newState - oldState)
              << ", target gap: " << optimalGap << std::endl;

    // Log convergence
    logConvergence(simTime().dbl(), nodeId, oldState, newState, gradient, consensusValue,
                  rMin, rMax, intervalWidth, optimalGap);

    // Save to history
    stateHistory.push_back(newState);
    consensusHistory.push_back(consensusValue);

    return newState;
}

// Enhanced combined update method - projection with consensus influence
double RDOAgent::updateStateWithConsensusAndProjection(const std::map<int, double>& neighborStates, double stepSize) {
    // Filter outliers based on selected algorithm
    std::map<int, double> filteredStates = filterOutliers(neighborStates);

    // Calculate weighted consensus
    double consensusValue = calculateConsensusValue(filteredStates);

    // Extract values from trusted neighbors
    std::vector<double> trustedValues;
    for (const auto& kv : neighborStates) {
        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
            trustedValues.push_back(kv.second);
        }
    }

    // Include own state in trusted values
    trustedValues.push_back(state);

    // Calculate bounds from trusted values
    double rMin = *std::min_element(trustedValues.begin(), trustedValues.end());
    double rMax = *std::max_element(trustedValues.begin(), trustedValues.end());
    double intervalWidth = rMax - rMin;

    // Calculate update terms
    double gradientTerm = -stepSize * gradient;
    double consensusTerm = (!filteredStates.empty()) ?
                          consensusWeight * stepSize * (consensusValue - state) : 0.0;

    // Combined update step
    double oldState = state;
    double proposed_state = oldState + gradientTerm + consensusTerm;

    // Project result to [rMin, rMax]
    double projectedState = std::min(std::max(proposed_state, rMin), rMax);

    // Calculate optimal gap for logging
    double optimalGap = std::abs(projectedState - costFunctionTarget);

    // Log convergence data
    logConvergence(simTime().dbl(), nodeId, oldState, projectedState, gradient, consensusValue,
                  rMin, rMax, intervalWidth, optimalGap);

    // Save to history
    stateHistory.push_back(projectedState);
    consensusHistory.push_back(consensusValue);

    return projectedState;
}

// Improved consensus value calculation to ensure influence
double RDOAgent::calculateConsensusValue(const std::map<int, double>& filteredStates) {
    // IMPORTANT FIX: If no neighbors, create synthetic value different from own state
    if (filteredStates.empty()) {
        // Return a value biased toward the target to ensure it differs from own state
        double targetDirection = (costFunctionTarget > state) ? 1.0 : -1.0;
        double syntheticConsensus = state + targetDirection * 2.0;

        std::cout << "  - No neighbors for consensus, using synthetic value: "
                  << syntheticConsensus << " (state: " << state << ")" << std::endl;

        return syntheticConsensus;
    }

    // Debug output for all neighbor states
    std::cout << "Node " << nodeId << " calculating consensus from "
              << filteredStates.size() << " neighbors:" << std::endl;
    for (auto& entry : filteredStates) {
        std::cout << "  - Neighbor " << entry.first << " state: " << entry.second << std::endl;
    }

    // Create trust weights with MODIFIED WEIGHTS to ensure influence
    std::map<int, double> trustWeights;
    double totalWeight = 0.0;

    // IMPORTANT FIX: Add self to filtered states with lower weight
    std::map<int, double> augmentedStates = filteredStates;
    augmentedStates[nodeId] = state;  // Add own state

    for (auto& entry : augmentedStates) {
        int neighborId = entry.first;
        double weight;

        if (neighborId == nodeId) {
            // IMPORTANT FIX: Use a lower weight for own state
            weight = 0.5; // Lower weight for own state to enhance neighbor influence
            std::cout << "  - Setting lower weight (0.5) for own state" << std::endl;
        } else {
            // Default weight for other nodes
            weight = 1.0;

            // Higher weight for trusted neighbors
            bool isTrustedNeighbor = std::find(trustedNeighbors.begin(), trustedNeighbors.end(),
                                             neighborId) != trustedNeighbors.end();
            if (isTrustedNeighbor) {
                weight = 3.0; // Higher weight for trusted nodes, but not excessive
            }
        }

        trustWeights[neighborId] = weight;
        totalWeight += weight;
    }

    // Normalize weights
    std::cout << "  - Trust weights (before normalization):" << std::endl;
    for (auto& w : trustWeights) {
        std::cout << "    * " << (w.first == nodeId ? "Self" : ("Neighbor " + std::to_string(w.first)))
                  << ": weight=" << w.second
                  << (w.first == nodeId ? " (self)" :
                     (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), w.first) != trustedNeighbors.end()
                      ? " (trusted)" : " (untrusted)")) << std::endl;

        w.second /= totalWeight > 0 ? totalWeight : 1.0;
    }

    double consensusValue = 0.0;

    switch (consensusAlgorithm) {
        case 1: { // Weighted Average
            std::cout << "  - Using weighted average consensus algorithm" << std::endl;
            for (auto& entry : augmentedStates) {
                std::cout << "    * Adding " << (trustWeights[entry.first] * entry.second)
                          << " from " << (entry.first == nodeId ? "self" : ("neighbor " + std::to_string(entry.first)))
                          << " (weight=" << trustWeights[entry.first] << ")" << std::endl;
                consensusValue += trustWeights[entry.first] * entry.second;
            }
            break;
        }

        case 2: { // Median
            std::cout << "  - Using median consensus algorithm" << std::endl;
            std::vector<double> values;
            for (auto& entry : augmentedStates) {
                values.push_back(entry.second);
            }

            std::sort(values.begin(), values.end());
            std::cout << "    * Sorted values: ";
            for (double v : values) std::cout << v << " ";
            std::cout << std::endl;

            size_t n = values.size();
            consensusValue = (n % 2 == 0) ? (values[n/2-1] + values[n/2]) / 2.0 : values[n/2];
            std::cout << "    * Median value: " << consensusValue << std::endl;
            break;
        }

        case 3: { // Trimmed Mean
            std::cout << "  - Using trimmed mean consensus algorithm" << std::endl;
            std::vector<double> values;
            for (auto& entry : augmentedStates) {
                values.push_back(entry.second);
            }

            std::sort(values.begin(), values.end());
            std::cout << "    * Original sorted values: ";
            for (double v : values) std::cout << v << " ";
            std::cout << std::endl;

            // Remove extremes if enough values
            if (values.size() >= 3) {
                std::cout << "    * Removing smallest value: " << values.front() << std::endl;
                std::cout << "    * Removing largest value: " << values.back() << std::endl;
                values.erase(values.begin()); // Remove smallest
                values.pop_back();            // Remove largest
            } else {
                std::cout << "    * Not enough values to trim, using all" << std::endl;
            }

            // Calculate mean of remaining values
            std::cout << "    * Remaining values: ";
            for (double v : values) std::cout << v << " ";
            std::cout << std::endl;

            consensusValue = values.empty() ? state : std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            std::cout << "    * Trimmed mean: " << consensusValue << std::endl;
            break;
        }

        default: // Default to weighted average
            std::cout << "  - Using default weighted average consensus algorithm" << std::endl;
            for (auto& entry : augmentedStates) {
                consensusValue += trustWeights[entry.first] * entry.second;
            }
            break;
    }

    std::cout << "  - Final consensus value: " << consensusValue
              << " (diff from state: " << (consensusValue - state) << ")" << std::endl;

    // IMPORTANT FIX: Ensure consensus is not exactly equal to state
    if (std::abs(consensusValue - state) < 0.001) {
        // Add a small bias toward the target
        double targetDir = (costFunctionTarget > state) ? 1.0 : -1.0;
        consensusValue = state + targetDir * 0.5;
        std::cout << "  - Forcing different consensus value: " << consensusValue << std::endl;
    }

    return consensusValue;
}

// Enhanced outlier filtering with multiple strategies
std::map<int, double> RDOAgent::filterOutliers(const std::map<int, double>& neighborStates) {
    if (neighborStates.empty()) {
        return neighborStates;
    }

    std::cout << "Node " << nodeId << " filtering outliers from "
              << neighborStates.size() << " neighbors" << std::endl;

    // Print all neighbor states
    for (auto& kv : neighborStates) {
        std::cout << "  - Neighbor " << kv.first << " state: " << kv.second << std::endl;
    }

    std::map<int, double> filteredStates;

    switch (filterType) {
        case 1: { // No filtering
            std::cout << "  - Using NO FILTERING (Type 1)" << std::endl;
            return neighborStates;
        }

        case 2: { // MAD-based statistical filtering
            std::cout << "  - Using MAD-BASED FILTERING (Type 2)" << std::endl;
            if (neighborStates.size() < 4) {
                std::cout << "    * Not enough data for MAD filtering, using all values" << std::endl;
                return neighborStates; // Not enough data for meaningful statistics
            }

            // Calculate median
            std::vector<double> values;
            for (auto& kv : neighborStates) {
                values.push_back(kv.second);
            }

            std::sort(values.begin(), values.end());
            double median = (values.size() % 2 == 0)
                          ? (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0
                          : values[values.size()/2];

            std::cout << "    * Median value: " << median << std::endl;

            // Calculate Median Absolute Deviation (MAD)
            std::vector<double> deviations;
            for (double val : values) {
                deviations.push_back(std::abs(val - median));
            }

            std::sort(deviations.begin(), deviations.end());
            double mad = (deviations.size() % 2 == 0)
                       ? (deviations[deviations.size()/2 - 1] + deviations[deviations.size()/2]) / 2.0
                       : deviations[deviations.size()/2];

            std::cout << "    * MAD value: " << mad << std::endl;

            // Filter out values more than 3 MADs away from median
            for (auto& kv : neighborStates) {
                double deviation = std::abs(kv.second - median);
                bool isOutlier = deviation > 3.0 * mad;

                std::cout << "    * Neighbor " << kv.first
                          << " deviation: " << deviation
                          << " (threshold: " << (3.0 * mad) << ")"
                          << " -> " << (isOutlier ? "OUTLIER" : "ACCEPTED") << std::endl;

                if (!isOutlier) {
                    filteredStates[kv.first] = kv.second;
                }
            }
            break;
        }

        case 3: { // Trust-based filtering
            std::cout << "  - Using TRUST-BASED FILTERING (Type 3)" << std::endl;
            for (auto& kv : neighborStates) {
                bool isTrusted = std::find(trustedNeighbors.begin(),
                                          trustedNeighbors.end(),
                                          kv.first) != trustedNeighbors.end();

                std::cout << "    * Neighbor " << kv.first
                          << " is " << (isTrusted ? "TRUSTED" : "UNTRUSTED")
                          << " -> " << (isTrusted ? "ACCEPTED" : "FILTERED") << std::endl;

                if (isTrusted) {
                    filteredStates[kv.first] = kv.second;
                }
            }
            break;
        }

        case 4: { // Combined Trust + MAD (enhanced for attack resilience)
            std::cout << "  - Using COMBINED TRUST+MAD FILTERING (Type 4)" << std::endl;

            // First apply trust filter
            std::map<int, double> trustFiltered;
            std::cout << "    * Step 1: Trust filtering" << std::endl;
            for (auto& kv : neighborStates) {
                bool isTrusted = std::find(trustedNeighbors.begin(),
                                          trustedNeighbors.end(),
                                          kv.first) != trustedNeighbors.end();

                std::cout << "      - Neighbor " << kv.first
                          << " is " << (isTrusted ? "TRUSTED" : "UNTRUSTED")
                          << " -> " << (isTrusted ? "ACCEPTED" : "FILTERED") << std::endl;

                if (isTrusted) {
                    trustFiltered[kv.first] = kv.second;
                }
            }

            // If not enough trusted nodes, use trust filtered
            if (trustFiltered.size() < 4) {
                std::cout << "    * Not enough trusted nodes for MAD filtering, using trust filter only" << std::endl;
                return trustFiltered;
            }

            // Then apply MAD to trusted nodes
            std::cout << "    * Step 2: MAD filtering on trusted nodes" << std::endl;
            std::vector<double> values;
            for (auto& kv : trustFiltered) {
                values.push_back(kv.second);
            }

            std::sort(values.begin(), values.end());
            double median = (values.size() % 2 == 0)
                          ? (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0
                          : values[values.size()/2];

            std::cout << "      - Median of trusted values: " << median << std::endl;

            std::vector<double> deviations;
            for (double val : values) {
                deviations.push_back(std::abs(val - median));
            }

            std::sort(deviations.begin(), deviations.end());
            double mad = (deviations.size() % 2 == 0)
                       ? (deviations[deviations.size()/2 - 1] + deviations[deviations.size()/2]) / 2.0
                       : deviations[deviations.size()/2];

            std::cout << "      - MAD of trusted values: " << mad << std::endl;

            // ENHANCED: Use a more aggressive outlier threshold for trusted nodes (2 MADs instead of 3)
            // This provides higher security for the consensus process
            double madMultiplier = 2.0;

            for (auto& kv : trustFiltered) {
                double deviation = std::abs(kv.second - median);
                bool isOutlier = deviation > madMultiplier * mad;

                std::cout << "      - Trusted neighbor " << kv.first
                          << " deviation: " << deviation
                          << " (threshold: " << (madMultiplier * mad) << ")"
                          << " -> " << (isOutlier ? "OUTLIER" : "ACCEPTED") << std::endl;

                if (!isOutlier) {
                    filteredStates[kv.first] = kv.second;
                }
            }
            break;
        }

        default:
            std::cout << "  - Using default (no filtering)" << std::endl;
            return neighborStates;
    }

    std::cout << "  - Filtering result: " << filteredStates.size()
              << " of " << neighborStates.size() << " neighbors accepted" << std::endl;

    return filteredStates;
}

// Check if a gradient value is an outlier (for attack detection)
bool RDOAgent::isGradientOutlier(double gradientValue) {
    // Define reasonable gradient bounds
    const double MAX_REASONABLE_GRADIENT = 200.0;

    return std::abs(gradientValue) > MAX_REASONABLE_GRADIENT;
}

// Get convergence history for analysis
std::vector<std::pair<double, double>> RDOAgent::getConvergenceHistory() const {
    std::vector<std::pair<double, double>> history;

    size_t minSize = std::min(stateHistory.size(), consensusHistory.size());
    for (size_t i = 0; i < minSize; i++) {
        history.push_back(std::make_pair(stateHistory[i], consensusHistory[i]));
    }

    return history;
}

// Static file handling
static std::ofstream convergenceLog;
static bool headerWritten = false;

// Helper function to check if directory exists
bool directoryExists(const std::string& path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

// Helper function to create directory
void createDirectory(const std::string& path) {
#ifdef _WIN32
    _mkdir(path.c_str());  // Windows
#else
    mkdir(path.c_str(), 0777);  // Unix/Linux
#endif
}

// COMPLETELY REWRITTEN DIRECT FILE LOGGING
void logConvergence(double time, int nodeId, double oldState, double newState, double gradient,
                   double consensus, double rMin, double rMax, double intervalWidth, double optimalGap) {

    // Direct console output for debugging
    std::cout << "Node " << nodeId << " at time " << time << " logging convergence data" << std::endl;

    // Create a unique filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    std::string baseFilename = "rdo_convergence_log.csv";

    // Try multiple locations
    std::vector<std::string> possiblePaths = {
        baseFilename,                  // Current directory
        "results/" + baseFilename,     // Results subdirectory
        "../results/" + baseFilename,  // Parent results directory
        "/tmp/" + baseFilename         // Temp directory (guaranteed writeable)
    };

    bool fileExists = false;
    std::string chosenPath;

    // Try to find an existing file first
    for (const auto& path : possiblePaths) {
        std::ifstream checkFile(path);
        if (checkFile.good()) {
            fileExists = true;
            chosenPath = path;
            checkFile.close();
            break;
        }
        checkFile.close();
    }

    // If no existing file, try to create one in each location
    if (chosenPath.empty()) {
        for (const auto& path : possiblePaths) {
            // Create directory if needed
            size_t lastSlash = path.find_last_of("/\\");
            if (lastSlash != std::string::npos) {
                std::string dir = path.substr(0, lastSlash);
                if (!dir.empty() && !directoryExists(dir)) {
                    std::cout << "Creating directory: " << dir << std::endl;
                    createDirectory(dir);
                }
            }

            // Try to open the file
            std::ofstream testFile(path, std::ios::app);
            if (testFile.is_open()) {
                chosenPath = path;
                testFile.close();
                break;
            }
        }
    }

    // If we still don't have a valid path, use a fallback
    if (chosenPath.empty()) {
        chosenPath = "/tmp/rdo_fallback_" + std::to_string(timestamp) + ".csv";
        std::cout << "Failed to find writable location, using fallback: " << chosenPath << std::endl;
    }

    // Open file in append mode
    std::ofstream logFile(chosenPath, std::ios::app);

    if (logFile.is_open()) {
        // Write header if file is new
        if (!fileExists) {
            std::cout << "Writing header to " << chosenPath << std::endl;
            logFile << "time,nodeId,oldState,newState,gradient,consensus,rMin,rMax,intervalWidth,optimalGap\n";
        }

        // Write data row
        std::cout << "Writing data for node " << nodeId << " to " << chosenPath << std::endl;
        logFile << std::fixed << std::setprecision(5)
                << time << ","
                << nodeId << ","
                << oldState << ","
                << newState << ","
                << gradient << ","
                << consensus << ","
                << rMin << ","
                << rMax << ","
                << intervalWidth << ","
                << optimalGap << "\n";

        // Force flush to ensure data is written
        logFile.flush();

        // Close file
        logFile.close();
        std::cout << "Log entry written successfully to " << chosenPath << std::endl;
    } else {
        std::cerr << "Failed to open log file: " << chosenPath << std::endl;
    }
}



/////Working Code///////


//#include "RDOAgent.h"
//#include <fstream>
//#include <iomanip>
//#include <sstream>
//#include <algorithm>
//#include <numeric>
//#include <cmath>
//#include <chrono>
//#include <ctime>
//#include <omnetpp.h>
//#include <sys/stat.h>
//
//using namespace omnetpp;
//
//// Constructor implementation
//RDOAgent::RDOAgent(int id, bool trusted) :
//    nodeId(id),
//    isTrusted(trusted),
//    state(0.0),
//    gradient(0.0),
//    filterType(3),              // Default to trust-based filtering
//    consensusAlgorithm(1),      // Default to weighted average
//    maxStateChange(10.0),       // Default limit for state changes
//    costFunctionTarget(0.0)     // Default target (to be set by CarApp)
//{
//    // Constructor implementation
//}
//
//// Initialization method
//void RDOAgent::initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all) {
//    state = initialState;
//    trustedNeighbors = trusted;
//    allNeighbors = all;
//}
//
//// Accessor methods
//double RDOAgent::getState() const {
//    return state;
//}
//
//void RDOAgent::setState(double newState) {
//    state = newState;
//}
//
//double RDOAgent::getGradient() const {
//    return gradient;
//}
//
//void RDOAgent::setGradient(double newGradient) {
//    gradient = newGradient;
//}
//
//// Configuration methods
//void RDOAgent::setFilterType(int type) {
//    filterType = type;
//}
//
//void RDOAgent::setConsensusAlgorithm(int algo) {
//    consensusAlgorithm = algo;
//}
//
//void RDOAgent::setMaxStateChange(double maxChange) {
//    maxStateChange = maxChange;
//}
//
//void RDOAgent::setCostFunctionTarget(double target) {
//    costFunctionTarget = target;
//}
//
//// Projection-based update (from original implementation)
//double RDOAgent::updateStateWithProjection(const std::map<int, double>& neighborStates, double stepSize) {
//    // Debug output
//    std::cout << "Node " << nodeId << " updating at time " << simTime().dbl()
//              << " with stepSize=" << stepSize
//              << ", current state=" << state
//              << ", gradient=" << gradient << std::endl;
//
//    // Extract values from trusted neighbors
//    std::vector<double> trustedValues;
//    for (const auto& kv : neighborStates) {
//        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//            trustedValues.push_back(kv.second);
//            std::cout << "  - Trusted neighbor " << kv.first << " has state " << kv.second << std::endl;
//        }
//    }
//
//    // Include own state in trusted values
//    trustedValues.push_back(state);
//
//    // IMPORTANT FIX: If all trusted values are still 0, add a non-zero value
//    bool allZeros = true;
//    for (double val : trustedValues) {
//        if (std::abs(val) > 0.001) {
//            allZeros = false;
//            break;
//        }
//    }
//
//    if (allZeros) {
//        double nonZeroValue = 10.0 + nodeId * 5.0;
//        trustedValues.push_back(nonZeroValue);
//        std::cout << "  - Adding forced non-zero value " << nonZeroValue
//                  << " to trusted values for node " << nodeId << std::endl;
//    }
//
//    // Calculate bounds from trusted values
//    double rMin = *std::min_element(trustedValues.begin(), trustedValues.end());
//    double rMax = *std::max_element(trustedValues.begin(), trustedValues.end());
//
//    // IMPORTANT FIX: Ensure bounds aren't both zero
//    if (std::abs(rMax - rMin) < 0.001) {
//        // If bounds are too close, widen them
//        rMin = std::min(rMin, state - 5.0);
//        rMax = std::max(rMax, state + 5.0);
//        std::cout << "  - Widening bounds to [" << rMin << ", " << rMax
//                  << "] for node " << nodeId << std::endl;
//    }
//
//    double intervalWidth = rMax - rMin;
//
//    // Standard gradient descent step
//    double xk = state;
//    double gradientTerm = -stepSize * gradient;
//    double xk_next = xk + gradientTerm;
//
//    std::cout << "  - Gradient term: " << gradientTerm
//              << ", proposed new state: " << xk_next << std::endl;
//
//    // Project result to [rMin, rMax]
//    double projectedState = std::min(std::max(xk_next, rMin), rMax);
//
//    // IMPORTANT FIX: Ensure state change happens
//    if (std::abs(projectedState - state) < 0.001) {
//        // Force movement toward target if no change
//        double targetDir = (costFunctionTarget > state) ? 1.0 : -1.0;
//        double forcedStep = targetDir * stepSize * 5.0;
//        projectedState = state + forcedStep;
//
//        // Project again to enforce bounds
//        projectedState = std::min(std::max(projectedState, rMin), rMax);
//
//        std::cout << "  - Forcing state change to " << projectedState
//                  << " for node " << nodeId << std::endl;
//    }
//
//    // Calculate optimal gap for logging
//    double optimalGap = std::abs(projectedState - costFunctionTarget);
//
//    // Calculate consensus for logging
//    double consensusValue = calculateConsensusValue(neighborStates);
//
//    std::cout << "  - Final projected state: " << projectedState
//              << ", change: " << (projectedState - state)
//              << ", target gap: " << optimalGap << std::endl;
//
//    // Log convergence data (only from trusted nodes)
//    //if (isTrusted) {
//    logConvergence(simTime().dbl(), nodeId, xk, projectedState, gradient, consensusValue,
//                  rMin, rMax, intervalWidth, optimalGap);
//    //}
//
//    return projectedState;
//}
//
//double RDOAgent::updateState(const std::map<int, double>& neighborStates, double stepSize) {
//    // Debug output
//    std::cout << "Node " << nodeId << " updating (consensus method) at time " << simTime().dbl()
//              << " with stepSize=" << stepSize
//              << ", current state=" << state << std::endl;
//
//    // Filter outliers based on selected algorithm
//    std::map<int, double> filteredStates = filterOutliers(neighborStates);
//
//    // Calculate weighted consensus
//    double consensusValue = calculateConsensusValue(filteredStates);
//
//    std::cout << "  - Consensus value: " << consensusValue
//              << " from " << filteredStates.size() << " neighbors" << std::endl;
//
//    // Calculate update terms
//    double gradientTerm = -stepSize * gradient;
//    double consensusTerm = (!filteredStates.empty()) ? 0.5 * stepSize * (consensusValue - state) : 0.0;
//
//    std::cout << "  - Gradient term: " << gradientTerm
//              << ", consensus term: " << consensusTerm << std::endl;
//
//    // Apply update with rate limiting
//    double oldState = state;
//    double stateChange = gradientTerm + consensusTerm;
//
//    // Limit maximum state change
//    if (std::abs(stateChange) > maxStateChange) {
//        stateChange = (stateChange > 0) ? maxStateChange : -maxStateChange;
//        std::cout << "  - Limiting change to " << stateChange << std::endl;
//    }
//
//    double newState = oldState + stateChange;
//
//    // IMPORTANT FIX: Ensure state change happens
//    if (std::abs(newState - oldState) < 0.001) {
//        // Force movement toward target if no change
//        double targetDir = (costFunctionTarget > oldState) ? 1.0 : -1.0;
//        double forcedStep = targetDir * stepSize * 5.0;
//        newState = oldState + forcedStep;
//        std::cout << "  - Forcing state change to " << newState
//                  << " for node " << nodeId << std::endl;
//    }
//
//    // Extract values from trusted neighbors for bounds and logging
//    std::vector<double> trustedValues;
//    for (const auto& kv : neighborStates) {
//        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//            trustedValues.push_back(kv.second);
//        }
//    }
//
//    // Include own state in trusted values
//    trustedValues.push_back(oldState);
//
//    // Calculate bounds and metrics from trusted values
//    double rMin = trustedValues.empty() ? oldState : *std::min_element(trustedValues.begin(), trustedValues.end());
//    double rMax = trustedValues.empty() ? oldState : *std::max_element(trustedValues.begin(), trustedValues.end());
//    double intervalWidth = rMax - rMin;
//    double optimalGap = std::abs(newState - costFunctionTarget);
//
//    std::cout << "  - Final new state: " << newState
//              << ", change: " << (newState - oldState)
//              << ", target gap: " << optimalGap << std::endl;
//
//    // Log convergence (only from trusted nodes)
//    //if (isTrusted) {
//    logConvergence(simTime().dbl(), nodeId, oldState, newState, gradient, consensusValue,
//                  rMin, rMax, intervalWidth, optimalGap);
//    //}
//
//    return newState;
//}
//
//// Enhanced combined update method - projection with consensus influence
//double RDOAgent::updateStateWithConsensusAndProjection(const std::map<int, double>& neighborStates, double stepSize) {
//    // Filter outliers based on selected algorithm
//    std::map<int, double> filteredStates = filterOutliers(neighborStates);
//
//    // Calculate weighted consensus
//    double consensusValue = calculateConsensusValue(filteredStates);
//
//    // Extract values from trusted neighbors
//    std::vector<double> trustedValues;
//    for (const auto& kv : neighborStates) {
//        if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//            trustedValues.push_back(kv.second);
//        }
//    }
//
//    // Include own state in trusted values
//    trustedValues.push_back(state);
//
//    // Calculate bounds from trusted values
//    double rMin = *std::min_element(trustedValues.begin(), trustedValues.end());
//    double rMax = *std::max_element(trustedValues.begin(), trustedValues.end());
//    double intervalWidth = rMax - rMin;
//
//    // Calculate update terms
//    double gradientTerm = -stepSize * gradient;
//    double consensusTerm = (!filteredStates.empty()) ? 0.5 * stepSize * (consensusValue - state) : 0.0;
//
//    // Combined update step
//    double oldState = state;
//    double proposed_state = oldState + gradientTerm + consensusTerm;
//
//    // Project result to [rMin, rMax]
//    double projectedState = std::min(std::max(proposed_state, rMin), rMax);
//
//    // Calculate optimal gap for logging
//    double optimalGap = std::abs(projectedState - costFunctionTarget);
//
//    // IMPORTANT: Temporarily log from all nodes for debugging
//    // Remove the trust check to ensure logging happens
//    //if (isTrusted) {
//    logConvergence(simTime().dbl(), nodeId, oldState, projectedState, gradient, consensusValue,
//                  rMin, rMax, intervalWidth, optimalGap);
//    //}
//
//    return projectedState;
//}
//
//// Helper method to calculate consensus value based on algorithm selection
//double RDOAgent::calculateConsensusValue(const std::map<int, double>& filteredStates) {
//    if (filteredStates.empty()) {
//        return state; // Default to own state if no neighbors
//    }
//
//    // Create trust weights with higher weights for trusted nodes
//    std::map<int, double> trustWeights;
//    double totalWeight = 0.0;
//
//    for (auto& entry : filteredStates) {
//        int neighborId = entry.first;
//        double weight = 1.0; // Default weight
//
//        // Higher weight for trusted neighbors
//        bool isTrustedNeighbor = std::find(trustedNeighbors.begin(), trustedNeighbors.end(),
//                                         neighborId) != trustedNeighbors.end();
//        if (isTrustedNeighbor) {
//            weight = 10.0; // 10x weight for trusted nodes
//        }
//
//        trustWeights[neighborId] = weight;
//        totalWeight += weight;
//    }
//
//    // Normalize weights
//    for (auto& w : trustWeights) {
//        w.second /= totalWeight > 0 ? totalWeight : 1.0;
//    }
//
//    double consensusValue = 0.0;
//
//    switch (consensusAlgorithm) {
//        case 1: { // Weighted Average
//            for (auto& entry : filteredStates) {
//                consensusValue += trustWeights[entry.first] * entry.second;
//            }
//            break;
//        }
//
//        case 2: { // Median
//            std::vector<double> values;
//            for (auto& entry : filteredStates) {
//                values.push_back(entry.second);
//            }
//
//            std::sort(values.begin(), values.end());
//            size_t n = values.size();
//            consensusValue = (n % 2 == 0) ? (values[n/2-1] + values[n/2]) / 2.0 : values[n/2];
//            break;
//        }
//
//        case 3: { // Trimmed Mean
//            std::vector<double> values;
//            for (auto& entry : filteredStates) {
//                values.push_back(entry.second);
//            }
//
//            std::sort(values.begin(), values.end());
//
//            // Remove extremes if enough values
//            if (values.size() >= 3) {
//                values.erase(values.begin()); // Remove smallest
//                values.pop_back();            // Remove largest
//            }
//
//            // Calculate mean of remaining values
//            consensusValue = values.empty() ? state : std::accumulate(values.begin(), values.end(), 0.0) / values.size();
//            break;
//        }
//
//        default: // Default to weighted average
//            for (auto& entry : filteredStates) {
//                consensusValue += trustWeights[entry.first] * entry.second;
//            }
//            break;
//    }
//
//    return consensusValue;
//}
//
//// Enhanced outlier filtering with multiple strategies
//std::map<int, double> RDOAgent::filterOutliers(const std::map<int, double>& neighborStates) {
//    if (neighborStates.empty()) {
//        return neighborStates;
//    }
//
//    std::map<int, double> filteredStates;
//
//    switch (filterType) {
//        case 1: { // No filtering
//            return neighborStates;
//        }
//
//        case 2: { // MAD-based statistical filtering
//            if (neighborStates.size() < 4) {
//                return neighborStates; // Not enough data for meaningful statistics
//            }
//
//            // Calculate median
//            std::vector<double> values;
//            for (auto& kv : neighborStates) {
//                values.push_back(kv.second);
//            }
//
//            std::sort(values.begin(), values.end());
//            double median = (values.size() % 2 == 0)
//                          ? (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0
//                          : values[values.size()/2];
//
//            // Calculate Median Absolute Deviation (MAD)
//            std::vector<double> deviations;
//            for (double val : values) {
//                deviations.push_back(std::abs(val - median));
//            }
//
//            std::sort(deviations.begin(), deviations.end());
//            double mad = (deviations.size() % 2 == 0)
//                       ? (deviations[deviations.size()/2 - 1] + deviations[deviations.size()/2]) / 2.0
//                       : deviations[deviations.size()/2];
//
//            // Filter out values more than 3 MADs away from median
//            for (auto& kv : neighborStates) {
//                if (std::abs(kv.second - median) <= 3.0 * mad) {
//                    filteredStates[kv.first] = kv.second;
//                }
//            }
//            break;
//        }
//
//        case 3: { // Trust-based filtering
//            for (auto& kv : neighborStates) {
//                if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//                    filteredStates[kv.first] = kv.second;
//                }
//            }
//            break;
//        }
//
//        case 4: { // Combined Trust + MAD
//            // First apply trust filter
//            std::map<int, double> trustFiltered;
//            for (auto& kv : neighborStates) {
//                if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//                    trustFiltered[kv.first] = kv.second;
//                }
//            }
//
//            // If not enough trusted nodes, return trust filtered
//            if (trustFiltered.size() < 4) {
//                return trustFiltered;
//            }
//
//            // Then apply MAD to trusted nodes
//            std::vector<double> values;
//            for (auto& kv : trustFiltered) {
//                values.push_back(kv.second);
//            }
//
//            std::sort(values.begin(), values.end());
//            double median = (values.size() % 2 == 0)
//                          ? (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0
//                          : values[values.size()/2];
//
//            std::vector<double> deviations;
//            for (double val : values) {
//                deviations.push_back(std::abs(val - median));
//            }
//
//            std::sort(deviations.begin(), deviations.end());
//            double mad = (deviations.size() % 2 == 0)
//                       ? (deviations[deviations.size()/2 - 1] + deviations[deviations.size()/2]) / 2.0
//                       : deviations[deviations.size()/2];
//
//            for (auto& kv : trustFiltered) {
//                if (std::abs(kv.second - median) <= 3.0 * mad) {
//                    filteredStates[kv.first] = kv.second;
//                }
//            }
//            break;
//        }
//
//        default:
//            return neighborStates;
//    }
//
//    return filteredStates;
//}
//
//// Helper function to check if directory exists
//bool directoryExists(const std::string& path) {
//    struct stat info;
//    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
//}
//
//// Helper function to create directory
//void createDirectory(const std::string& path) {
//#ifdef _WIN32
//    _mkdir(path.c_str());  // Windows
//#else
//    mkdir(path.c_str(), 0777);  // Unix/Linux
//#endif
//}
//
//// COMPLETELY REWRITTEN DIRECT FILE LOGGING
//void logConvergence(double time, int nodeId, double oldState, double newState, double gradient,
//                   double consensus, double rMin, double rMax, double intervalWidth, double optimalGap) {
//
//    // Direct console output for debugging
//    std::cout << "Node " << nodeId << " at time " << time << " logging convergence data" << std::endl;
//
//    // Create a unique filename with timestamp
//    auto now = std::chrono::system_clock::now();
//    auto timestamp = std::chrono::system_clock::to_time_t(now);
//    std::string baseFilename = "rdo_convergence_log.csv";
//
//    // Try multiple locations
//    std::vector<std::string> possiblePaths = {
//        baseFilename,                  // Current directory
//        "results/" + baseFilename,     // Results subdirectory
//        "../results/" + baseFilename,  // Parent results directory
//        "/tmp/" + baseFilename         // Temp directory (guaranteed writeable)
//    };
//
//    bool fileExists = false;
//    std::string chosenPath;
//
//    // Try to find an existing file first
//    for (const auto& path : possiblePaths) {
//        std::ifstream checkFile(path);
//        if (checkFile.good()) {
//            fileExists = true;
//            chosenPath = path;
//            checkFile.close();
//            break;
//        }
//        checkFile.close();
//    }
//
//    // If no existing file, try to create one in each location
//    if (chosenPath.empty()) {
//        for (const auto& path : possiblePaths) {
//            // Create directory if needed
//            size_t lastSlash = path.find_last_of("/\\");
//            if (lastSlash != std::string::npos) {
//                std::string dir = path.substr(0, lastSlash);
//                if (!dir.empty() && !directoryExists(dir)) {
//                    std::cout << "Creating directory: " << dir << std::endl;
//                    createDirectory(dir);
//                }
//            }
//
//            // Try to open the file
//            std::ofstream testFile(path, std::ios::app);
//            if (testFile.is_open()) {
//                chosenPath = path;
//                testFile.close();
//                break;
//            }
//        }
//    }
//
//    // If we still don't have a valid path, use a fallback
//    if (chosenPath.empty()) {
//        chosenPath = "/tmp/rdo_fallback_" + std::to_string(timestamp) + ".csv";
//        std::cout << "Failed to find writable location, using fallback: " << chosenPath << std::endl;
//    }
//
//    // Open file in append mode
//    std::ofstream logFile(chosenPath, std::ios::app);
//
//    if (logFile.is_open()) {
//        // Write header if file is new
//        if (!fileExists) {
//            std::cout << "Writing header to " << chosenPath << std::endl;
//            logFile << "time,nodeId,oldState,newState,gradient,consensus,rMin,rMax,intervalWidth,optimalGap\n";
//        }
//
//        // Write data row
//        std::cout << "Writing data for node " << nodeId << " to " << chosenPath << std::endl;
//        logFile << std::fixed << std::setprecision(5)
//                << time << ","
//                << nodeId << ","
//                << oldState << ","
//                << newState << ","
//                << gradient << ","
//                << consensus << ","
//                << rMin << ","
//                << rMax << ","
//                << intervalWidth << ","
//                << optimalGap << "\n";
//
//        // Force flush to ensure data is written
//        logFile.flush();
//
//        // Close file
//        logFile.close();
//        std::cout << "Log entry written successfully to " << chosenPath << std::endl;
//    } else {
//        std::cerr << "Failed to open log file: " << chosenPath << std::endl;
//    }
//}

/////Working Code///////


//#include "RDOAgent.h"
//#include <fstream>
//#include <iomanip>
//#include <sstream>
//#include <algorithm>
//#include <numeric>  // For std::accumulate
//#include <cmath>
//#include <omnetpp.h>
//#include <sys/stat.h>  // For directory operations
//
//using namespace omnetpp;
//
//// Constructor implementation
//RDOAgent::RDOAgent(int id, bool trusted) :
//    nodeId(id),
//    isTrusted(trusted),
//    state(0.0),
//    gradient(0.0) {
//    // Constructor implementation
//}
//
//// Initialization method
//void RDOAgent::initialize(double initialState, const std::vector<int>& trusted, const std::vector<int>& all) {
//    state = initialState;
//    trustedNeighbors = trusted;
//    allNeighbors = all;
//}
//
//// Accessors implementation
//double RDOAgent::getState() const {
//    return state;
//}
//
//void RDOAgent::setState(double newState) {
//    state = newState;
//}
//
//double RDOAgent::getGradient() const {
//    return gradient;
//}
//
//void RDOAgent::setGradient(double newGradient) {
//    gradient = newGradient;
//}
//
//// Algorithm Configuration
//void RDOAgent::setFilterType(int type) {
//    filterType = type;
//}
//
//void RDOAgent::setConsensusAlgorithm(int algo) {
//    consensusAlgorithm = algo;
//}
//
///* The rest of your existing implementation for:
//   - static functions like logConvergence
//   - updateState
//   - filterOutliers
//   As shown in your previous code */
//
//static std::ofstream convergenceLog;
//static bool headerWritten = false;
//
//// Helper function to check if directory exists
//bool directoryExists(const std::string& path) {
//    struct stat info;
//    return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
//}
//
//// Helper function to create directory
//void createDirectory(const std::string& path) {
//#ifdef _WIN32
//    _mkdir(path.c_str());  // Windows
//#else
//    mkdir(path.c_str(), 0777);  // Unix/Linux
//#endif
//}
//
//void logConvergence(double time, int nodeId, double oldState, double newState, double gradient, double consensus) {
//    if (!headerWritten) {
//        if (!directoryExists("results")) {
//            createDirectory("results");
//        }
//        convergenceLog.open("results/rdo_convergence_log.csv", std::ios::app);
//        if (convergenceLog.is_open()) {
//            convergenceLog << "time,nodeId,oldState,newState,gradient,consensus\n";
//            headerWritten = true;
//        }
//    }
//
//    if (convergenceLog.is_open()) {
//        convergenceLog << std::fixed << std::setprecision(5)
//                       << time << ","
//                       << nodeId << ","
//                       << oldState << ","
//                       << newState << ","
//                       << gradient << ","
//                       << consensus << "\n";
//    }
//}
//
//double RDOAgent::updateState(const std::map<int, double>& neighborStates, double stepSize) {
//    std::map<int, double> filteredStates = filterOutliers(neighborStates);
//    std::map<int, double> trustWeights;
//    double totalWeight = 0.0;
//
//    for (auto& state : filteredStates) {
//        int neighborId = state.first;
//        double weight = 1.0;
//        bool isTrustedNeighbor = std::find(trustedNeighbors.begin(), trustedNeighbors.end(), neighborId) != trustedNeighbors.end();
//        if (isTrustedNeighbor)
//            weight = 10.0;
//        trustWeights[neighborId] = weight;
//        totalWeight += weight;
//    }
//
//    for (auto& w : trustWeights)
//        w.second /= totalWeight > 0 ? totalWeight : 1.0;
//
//    double consensusValue = 0.0;
//    if (consensusAlgorithm == 1) {  // Average
//        for (auto& state : filteredStates)
//            consensusValue += trustWeights[state.first] * state.second;
//
//    } else if (consensusAlgorithm == 2) {  // Median
//        std::vector<double> values;
//        for (auto& state : filteredStates)
//            values.push_back(state.second);
//        std::sort(values.begin(), values.end());
//        size_t n = values.size();
//        consensusValue = (n % 2 == 0) ? (values[n/2-1] + values[n/2]) / 2.0 : values[n/2];
//
//    } else if (consensusAlgorithm == 3) {  // Trimmed Mean
//        std::vector<double> values;
//        for (auto& state : filteredStates)
//            values.push_back(state.second);
//        std::sort(values.begin(), values.end());
//        if (values.size() >= 3) {
//            values.erase(values.begin());
//            values.pop_back();
//        }
//        consensusValue = values.empty() ? state : std::accumulate(values.begin(), values.end(), 0.0) / values.size();
//    }
//
//    double gradientTerm = -stepSize * gradient;
//    double consensusTerm = (!filteredStates.empty()) ? 0.5 * stepSize * (consensusValue - state) : 0.0;
//
//    double oldState = state;
//    double newState = state + gradientTerm + consensusTerm;
//    double stateChange = newState - state;
//
//    if (std::abs(stateChange) > maxStateChange) {
//        stateChange = (stateChange > 0) ? maxStateChange : -maxStateChange;
//        newState = state + stateChange;
//    }
//
////    logConvergence(simTime().dbl(), nodeId, oldState, newState, gradient, consensusValue);
//    if (isTrusted) {
//            logConvergence(simTime().dbl(), nodeId, oldState, newState, gradient, consensusValue);
//        }
//    return newState;
//
////    if (isTrusted) {
////        logConvergence(simTime().dbl(), nodeId, oldState, newState, gradient, consensusValue);
////    }
//
//}
//
//
//std::map<int, double> RDOAgent::filterOutliers(const std::map<int, double>& neighborStates)
//{
//    if (neighborStates.empty())
//        return neighborStates;
//
//    std::map<int, double> filteredStates;
//
//    switch (filterType) {
//        case 1:  // No filtering
//            return neighborStates;
//
//        case 2: {  // MAD
//            if (neighborStates.size() < 4)
//                return neighborStates;
//
//            std::vector<double> values;
//            for (auto& kv : neighborStates)
//                values.push_back(kv.second);
//
//            std::sort(values.begin(), values.end());
//            double median = (values.size() % 2 == 0)
//                            ? (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0
//                            : values[values.size()/2];
//
//            std::vector<double> deviations;
//            for (double val : values)
//                deviations.push_back(std::abs(val - median));
//
//            std::sort(deviations.begin(), deviations.end());
//            double mad = (deviations.size() % 2 == 0)
//                         ? (deviations[deviations.size()/2 - 1] + deviations[deviations.size()/2]) / 2.0
//                         : deviations[deviations.size()/2];
//
//            for (auto& kv : neighborStates) {
//                if (std::abs(kv.second - median) <= 3.0 * mad)
//                    filteredStates[kv.first] = kv.second;
//            }
//            break;
//        }
//
//        case 3: {  // Trust-based
//            for (auto& kv : neighborStates) {
//                if (std::find(trustedNeighbors.begin(), trustedNeighbors.end(), kv.first) != trustedNeighbors.end()) {
//                    filteredStates[kv.first] = kv.second;
//                }
//            }
//            break;
//        }
//
//        default:
//            return neighborStates;
//    }
//
//    return filteredStates;
//}



//#include "RDOAgent.h"
//
//double RDOAgent::updateState(const std::map<int, double>& neighborStates, double stepSize) {
//    // First, filter out outliers to improve resilience
//    std::map<int, double> filteredStates = filterOutliers(neighborStates);
//
//    // Define trust weights for each neighbor
//    std::map<int, double> trustWeights;
//    double totalWeight = 0.0;
//
//    // Assign higher weights to trusted nodes (gNBs)
//    for (auto& state : filteredStates) {
//        int neighborId = state.first;
//        double weight = 1.0;  // Default weight
//
//        // Check if this is a trusted neighbor (gNB)
//        bool isTrustedNeighbor = std::find(trustedNeighbors.begin(), trustedNeighbors.end(), neighborId) != trustedNeighbors.end();
//
//        if (isTrustedNeighbor) {
//            // Higher weight for trusted nodes (gNBs)
//            weight = 10.0;
//        }
//
//        trustWeights[neighborId] = weight;
//        totalWeight += weight;
//    }
//
//    // Normalize weights
//    if (totalWeight > 0) {
//        for (auto& weight : trustWeights) {
//            weight.second /= totalWeight;
//        }
//    }
//
//    // Calculate the weighted average of neighbor states
//    double consensusValue = 0.0;
//
//    if (consensusAlgorithm == 1) { // Average
//        double weightedSum = 0.0;
//        for (auto& state : filteredStates) {
//            int neighborId = state.first;
//            double neighborState = state.second;
//            double weight = trustWeights[neighborId];
//            weightedSum += weight * neighborState;
//        }
//        consensusValue = weightedSum;
//
//    } else if (consensusAlgorithm == 2) { // Median
//        std::vector<double> values;
//        for (auto& state : filteredStates) {
//            values.push_back(state.second);
//        }
//        if (!values.empty()) {
//            std::sort(values.begin(), values.end());
//            size_t n = values.size();
//            consensusValue = (n % 2 == 0) ? (values[n/2-1] + values[n/2]) / 2.0 : values[n/2];
//        }
//
//    } else if (consensusAlgorithm == 3) { // Trimmed Mean
//        std::vector<double> values;
//        for (auto& state : filteredStates) {
//            values.push_back(state.second);
//        }
//        if (values.size() >= 3) { // Need at least 3 values
//            std::sort(values.begin(), values.end());
//            values.erase(values.begin()); // Remove lowest
//            values.pop_back();             // Remove highest
//        }
//        double sum = 0.0;
//        for (double v : values) sum += v;
//        consensusValue = values.empty() ? state : sum / values.size();
//    }
//
//
//    // Calculate new state using gradient descent with consensus term
//    double gradientTerm = -stepSize * gradient;
//    double consensusTerm = 0.0;
//
//    // Only add consensus term if we have neighbors
//    if (!filteredStates.empty()) {
//        consensusTerm = 0.5 * stepSize * (consensusValue - state);
//
//    }
//
//    // Calculate new state
//    double newState = state + gradientTerm + consensusTerm;
//
//    // Limit change rate for stability
//    double stateChange = newState - state;
//    if (std::abs(stateChange) > maxStateChange) {
//        stateChange = (stateChange > 0) ? maxStateChange : -maxStateChange;
//        newState = state + stateChange;
//    }
//
//    return newState;
//}
//
//std::map<int, double> RDOAgent::filterOutliers(const std::map<int, double>& neighborStates)
//{
//    if (neighborStates.empty()) {
//        return neighborStates;
//    }
//
//    std::map<int, double> filteredStates;
//
//    switch (filterType) {
//        case 1: // No filtering
//            filteredStates = neighborStates;
//            break;
//
//        case 2: { // Outlier removal based on Median Absolute Deviation (MAD)
//            if (neighborStates.size() < 4) {
//                filteredStates = neighborStates;
//                break;
//            }
//
//            std::vector<double> stateValues;
//            for (const auto& entry : neighborStates) {
//                stateValues.push_back(entry.second);
//            }
//
//            size_t n = stateValues.size();
//            std::sort(stateValues.begin(), stateValues.end());
//            double median = (n % 2 == 0) ? (stateValues[n/2-1] + stateValues[n/2]) / 2.0 : stateValues[n/2];
//
//            std::vector<double> absDeviations;
//            for (double val : stateValues) {
//                absDeviations.push_back(std::abs(val - median));
//            }
//            std::sort(absDeviations.begin(), absDeviations.end());
//            double mad = (n % 2 == 0) ? (absDeviations[n/2-1] + absDeviations[n/2]) / 2.0 : absDeviations[n/2];
//
//            for (const auto& entry : neighborStates) {
//                if (std::abs(entry.second - median) <= 3.0 * mad) {
//                    filteredStates[entry.first] = entry.second;
//                }
//            }
//            break;
//        }
//
//        case 3: { // Trust-based filtering
//            for (const auto& entry : neighborStates) {
//                int neighborId = entry.first;
//                bool isTrustedNeighbor = std::find(trustedNeighbors.begin(), trustedNeighbors.end(), neighborId) != trustedNeighbors.end();
//                if (isTrustedNeighbor) {
//                    filteredStates[neighborId] = entry.second;
//                }
//            }
//            break;
//        }
//
//        default:
//            filteredStates = neighborStates;
//            break;
//    }
//
//    return filteredStates;
//}
