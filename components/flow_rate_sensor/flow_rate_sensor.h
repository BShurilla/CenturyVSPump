#include "esphome.h"

class FlowRateSensor : public PollingComponent, public Sensor {
 public:
  FlowRateSensor() : PollingComponent(1000) {}

  void setup() override {
    pinMode(4, INPUT);  // GPIO4 (Q0.0)
    attachInterrupt(digitalPinToInterrupt(4), FlowRateSensor::flowPulseCounter, FALLING);
  }

  void update() override {
    float flowRate = read_flow_rate();
    publish_state(flowRate);
  }

  Sensor *get_flow_sensor() { return this; }

 private:
  static volatile byte flowPulseCount;
  unsigned long oldTime = 0;
  const float flowCalibrationFactor = 0.2;

  static void IRAM_ATTR flowPulseCounter() {
    flowPulseCount++;
  }

  float read_flow_rate() {
    float flowRateGPH;
    if((millis() - oldTime) > 900) {
      detachInterrupt(digitalPinToInterrupt(4));
      flowRateGPH = ((1000.0 / (millis() - oldTime)) * flowPulseCount) / flowCalibrationFactor * 15.8503;
      oldTime = millis();
      flowPulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(4), FlowRateSensor::flowPulseCounter, FALLING);
    }
    return flowRateGPH;
  }
};

volatile byte FlowRateSensor::flowPulseCount = 0;
