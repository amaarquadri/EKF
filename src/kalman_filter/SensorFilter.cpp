#include "SensorFilter.h"

#include "Vector.h"

void SensorFilter::updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                                 const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt) {
    Vector<16> doubleZ{doubleSensorMeasurements};
    Vector<1, uint8_t> uint8Z{uint8SensorMeasurements};
    Vector<1, bool> boolZ{(bool) boolSensorMeasurements[0]};
    Vector<4> control_inputsVec{control_inputs};
    SensorMeasurements sensorMeasurements = SensorMeasurements<>::parseZ(doubleZ, uint8Z, boolZ);
    update(sensorMeasurements, ControlInputs::parseU(control_inputsVec), dt);
}

void SensorFilter::getOutputWrapper(double *doubleAircraftState) const {
    Vector<19> aircraftState = getOutput().getX();
    for (size_t i = 0; i < 19; i++) doubleAircraftState[i] = aircraftState[i];
}
