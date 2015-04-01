#ifndef ORC_TOUCH_H
#define ORC_TOUCH_H

#include "orc/status.h"
#include "orc/joints.h"

#define PORT_GRIPPER 2

// Class copied straight from DigitalInput.java from the uorc library.
//
class DigitalInput {
private:
    int port;
    bool invert;
    OrcStatus* ost;
public:
    DigitalInput(OrcStatus* ost, int port, bool pullup, bool invert)
        :ost(ost),
         port(port),
         invert(invert) {
        uint32_t command = 0;
        uint8_t buf[] = {0,0,0,0,0,0};
        size_t size = 0;
        if (port < 8) {
            command = 0x6000;
            buf[0] = (uint8_t)port;
            buf[1] = 1;
            buf[2] = (uint8_t)(pullup?1:0);
            size = 3;
        } else {
            command = 0x7000;
            buf[0] = (uint8_t) (port-8);
            buf[1] = FAST_DIGIO_MODE_IN;
            size = 6; // rest are zeros, assuming they're set that way.
        }
        uorc_response* resp = nullptr;
        do {
            if(resp != nullptr) uorc_response_destroy(resp);
            resp = uorc_command(ost->uorc(), 0x7000, buf, size, -1);
        } while(!resp->valid);
        uorc_response_destroy(resp);
    }

    bool getValue();
};

void monitor_touch(OrcStatus*);

#endif
