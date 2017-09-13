#include "Arduino.h"
#ifndef MicroBees_h
#define MicroBees_h
#include <avr/wdt.h>
#define ADC_BITS    10
#define ADC_COUNTS  (1<<ADC_BITS)
#define PHASECAL  0
#define VCAL  533
#define ICAL  7.35
#define I_PIN  A1 
#define V_PIN  A0
class WireDuino
{
  public:
    WireDuino();
    void turnOnRelay();
    void turnOffRelay();
	void setRelay(uint8_t target_state);
	void loop();
	static double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
	static double realPowerAV,
       apparentPowerAV,
       powerFactorAV,
       VrmsAV,
       IrmsAV;
  private:
	void powerMetering();
};
#endif
