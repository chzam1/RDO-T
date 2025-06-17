/*
 * RDOManager.h
 *
 *  Created on: Apr 13, 2025
 *      Author: simu5g
 */

#ifndef RDOEXPERIMENT_SRC_RDOMANAGER_H_
#define RDOEXPERIMENT_SRC_RDOMANAGER_H_


#include <omnetpp.h>
#include <vector>
#include <string>

using namespace omnetpp;

class RDOManager : public cSimpleModule
{
  private:
    std::vector<int> trustedAgents;  // List of trusted gNB IDs

  protected:
    virtual void initialize() override;

  private:
    std::vector<int> loadTrustedAgents(const std::string& filePath);
    void validateTrustedAgents();
};


#endif /* RDOEXPERIMENT_SRC_RDOMANAGER_H_ */
