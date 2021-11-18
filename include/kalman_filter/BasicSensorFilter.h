#pragma once

#include "SensorFilter.h"

class BasicSensorFilter : public SensorFilter {
private:
    AircraftState state;
public:
    BasicSensorFilter();

    void update(const SensorMeasurements<> &sensorMeasurements,
                const ControlInputs &control_inputs, const double &dt) override;

    [[nodiscard]] AircraftState getOutput() const override;
};
