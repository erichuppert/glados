#ifndef ORC_PUB_STATUS_H
#define ORC_PUB_STATUS_H

#include "orc/uorc.h"
#include <mutex>

class AutoLock {
private:
    std::mutex* mtx;
public:
    AutoLock(std::mutex* l) {
        mtx = l;
        mtx->lock();
    }
    ~AutoLock() {
        mtx->unlock();
    }
};

#define STATUS_RATE 40

class OrcStatus {
private:
    uorc_status_t status;
    uorc_status_t status_copy;
    uorc_t* _uorc;
    std::mutex mtx;
public:
    bool status_received;
    OrcStatus(uorc_t* uorc);

    void set();

    uorc_status_t get();

    uorc_t* uorc();
};

void updateStatus(OrcStatus*);
#endif
