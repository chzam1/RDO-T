//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// TODO generated message class
//
//packet RDOMessage {
//    int someField;
//    string anotherField;
//    double arrayField1[];
//    double arrayField2[10];
//}

//
// RDO Message Definitions for Resilient Distributed Optimization
//

cplusplus {{
#include "inet/common/INETDefs.h"
}}

namespace inet;

//
// Base packet for RDO communication
//
packet RDOPacket {
    int sourceId;           // ID of the sending node
    int destinationId = -1; // ID of the destination node (-1 for broadcast)
    int sequenceNumber;     // Sequence number for message ordering
    simtime_t timestamp;    // Timestamp when message was created
}

//
// Packet containing state information for RDO
//
packet RDOStatePacket extends RDOPacket {
    double stateValue;      // State value
}

//
// Packet for RDO gradient information
//
packet RDOGradientPacket extends RDOPacket {
    double gradientValue;   // Gradient value
}