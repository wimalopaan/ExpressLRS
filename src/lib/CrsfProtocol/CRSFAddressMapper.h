#pragma once

#include "crsf_protocol.h"

#if defined(WMEXTENSION) && defined(TARGET_TX) && defined(WMRELAY)
class CRSFAddressMapper {
public:
    inline crsf_addr_e mapOutgoingAddress(const crsf_addr_e addr) const {
        switch (addr) {
            case CRSF_ADDRESS_CRSF_TRANSMITTER:  return CRSF_ADDRESS_CRSF_REPEATER_TRANSMITTER;
            case CRSF_ADDRESS_CRSF_RECEIVER:     return CRSF_ADDRESS_CRSF_REPEATER_RECEIVER;
            default:                             return addr;
        }
    }

    inline crsf_addr_e mapIncomingAddress(const crsf_addr_e addr) const {
        switch (addr) {
        case CRSF_ADDRESS_CRSF_REPEATER_TRANSMITTER: 
            return CRSF_ADDRESS_CRSF_TRANSMITTER; 
            break;
        case CRSF_ADDRESS_CRSF_REPEATER_RECEIVER: 
            return CRSF_ADDRESS_CRSF_RECEIVER; 
            break;
        default:                                
            return addr;
            break;
        }
    }

    inline bool shouldRemapAddress(crsf_addr_e addr) const {
        return (addr == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
               (addr == CRSF_ADDRESS_CRSF_RECEIVER) ||
               (addr == CRSF_ADDRESS_CRSF_REPEATER_TRANSMITTER) ||
               (addr == CRSF_ADDRESS_CRSF_REPEATER_RECEIVER);
    }
};
#endif
