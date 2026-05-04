#pragma once

#include <Eigen/Dense>

namespace sensors {

class SensorBase {
public:
    virtual ~SensorBase() = default;
    virtual void set_noise(bool enabled) = 0;
    virtual bool is_updated() const { return updated_; }

protected:
    bool enable_noise_ = true;
    bool updated_ = false;
};

} // namespace sensors
