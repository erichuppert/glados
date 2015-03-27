#ifndef ORC_PUB_STATUS_H
#define ORC_PUB_STATUS_H

#include "orc_utils/uorc.h"

#define STATUS_RATE 40

class OrcStatus {
private:
    uorc_status_t status;
    uorc_status_t status_copy;
    uorc_t* uorc;
    std::mutex mtx;
public:
    OrcStatus(uorc_t* uorc);

    void set();

    uorc_status_t& get();

    uorc_t* uorc();
}
#endif
