/*
 * RDOManager.cc
 *
 *  Created on: Apr 13, 2025
 *      Author: simu5g
 */


#include "RDOManager.h"
#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace inet;

Define_Module(RDOManager);

void RDOManager::initialize()
{
    EV_INFO << "Initializing RDOManager..." << endl;

    // Load trusted agents from config file (CSV)
    std::string filePath = par("configFile").stringValue();

    if (!filePath.empty()) {
        trustedAgents = loadTrustedAgents(filePath);
        EV_INFO << "Loaded " << trustedAgents.size() << " trusted agents from " << filePath << endl;

        if (par("validateCDS").boolValue()) {
            validateTrustedAgents();
        }
    } else {
        EV_WARN << "No RDO config file specified!" << endl;
    }
}

std::vector<int> RDOManager::loadTrustedAgents(const std::string& filePath)
{
    std::vector<int> agents;
    std::ifstream file(filePath);

    if (!file.is_open()) {
        EV_ERROR << "Cannot open RDO config file: " << filePath << endl;
        return agents;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;

        // Assuming each line is just a gNB ID
        if (std::getline(ss, token, ',')) {
            try {
                int agentId = std::stoi(token);
                agents.push_back(agentId);
            } catch (std::exception& e) {
                EV_WARN << "Invalid line in RDO config: " << line << endl;
            }
        }
    }
    file.close();
    return agents;
}

void RDOManager::validateTrustedAgents()
{
    if (trustedAgents.empty()) {
        EV_WARN << "No trusted agents loaded. Cannot validate CDS." << endl;
        return;
    }

    // A very basic validation (advanced: check actual network graph connectivity)
    EV_INFO << "Trusted Agents (IDs): ";
    for (auto id : trustedAgents) {
        EV_INFO << id << " ";
    }
    EV_INFO << endl;

    // You can add more formal CDS validation here if needed
}


