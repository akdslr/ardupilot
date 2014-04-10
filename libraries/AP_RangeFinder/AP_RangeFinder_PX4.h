/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_RangeFinder_PX4_H
#define AP_RangeFinder_PX4_H

#include "RangeFinder.h"

class AP_RangeFinder_PX4 : public RangeFinder
{
public:
    // constructor
    AP_RangeFinder_PX4(FilterInt16 *filter);
    
    // initialize all the range finder devices
    bool init(void);
    
    bool take_reading(void);
    
    void accumulate(void);
    
    // read value from primary sensor and return distance in cm
    int16_t read();
    
    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }
private:
    uint8_t _get_primary(void) const;
    uint8_t _num_instances;
    int _range_fd[RANGEFINDER_MAX_INSTANCES];
    float _sum[RANGEFINDER_MAX_INSTANCES];
    uint32_t _count[RANGEFINDER_MAX_INSTANCES];
    uint64_t _last_timestamp[RANGEFINDER_MAX_INSTANCES];
};

#endif // AP_RangeFinder_PX4_H

