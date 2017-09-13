#include "Arduino.h"
#include "MicroBees.h"
double WireDuino::realPower = 0;
double WireDuino::apparentPower = 0;
double WireDuino::powerFactor = 0;
double WireDuino::Vrms = 0;
double WireDuino:: Irms = 0;
double WireDuino::realPowerAV = 0;
double WireDuino::apparentPowerAV = 0;
double WireDuino::powerFactorAV = 0;
double WireDuino::VrmsAV = 0;
double WireDuino::IrmsAV = 0;


WireDuino::WireDuino()
{
  cli();  
  wdt_enable(WDTO_8S); 
  sei();
  wdt_reset();
  pinMode(5, OUTPUT);
  
}

void WireDuino::turnOnRelay()
{
  digitalWrite(5, HIGH); 
}

void WireDuino::turnOffRelay()
{
  digitalWrite(5, LOW); 
}

void WireDuino::setRelay(uint8_t target_state)
{
  digitalWrite(5, target_state>0 ? HIGH : LOW); 
}

static long readVcc() {
  long result;

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
	
  #endif

  #if defined(__AVR__) 
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;
  return result;
 #elif defined(__arm__)
  return (3300);
 #else 
  return (3300);
 #endif
}



void WireDuino::powerMetering(){
	int SUPPLYVOLTAGE = readVcc();
    analogReference(DEFAULT);
        
    double phaseShiftedV;
  
    int crossings = 20;
    long timeout = 20000;
  
    int crossCount = 0;
    int numberOfSamples = 0;
    
    int sampleV, lastSampleV;
    double filteredV, lastFilteredV;
    int sampleI, lastSampleI;
    double filteredI, lastFilteredI;
    
    double sqV = 0;
    double sumSqV = 0;
    double sumV = 0;
    double sqI = 0;
    double sumSqI = 0;
    double sumI = 0;
    double instP = 0;
    double sumP = 0;
    double meanV = 0;
    double meanI = 0;

    int startV;
    
    boolean lastVCross, checkVCross;
    
    boolean st=false;
    unsigned long start = millis();
  
    boolean wasVNegative = false;
  
    while(st==false) {
       wdt_reset();
       startV = analogRead(V_PIN);
       
       if(wasVNegative) {
         if(startV > 511){
           st = true;
         }
       } else {
         wasVNegative = (startV <= 511);
       }
       if ((millis()-start)>timeout) st = true;
    }

    //---------Calculation of the V and I offset
    while (crossCount < crossings) {
      wdt_reset();
      numberOfSamples++;                            //Count number of times looped.
      sampleV = analogRead(V_PIN);                 //Read in raw voltage signal
      sampleI = analogRead(I_PIN);                 //Read in raw current signal
      sumV += sampleV;
      sumI += sampleI;
      lastVCross = checkVCross;                     
      if (sampleV > startV) checkVCross = true; 
                       else checkVCross = false;
      if (numberOfSamples==1) lastVCross = checkVCross;                  
                       
      if (lastVCross != checkVCross) crossCount++;
    }
    
    meanV = sumV / numberOfSamples;
    meanI = sumI / numberOfSamples;

    numberOfSamples = 0;
    filteredV = 0;
    filteredI = 0;
    crossCount = 0;
    checkVCross = false;
    
    //-------------------------------------------------------------------------------------------------------------------------
    // 2) Main measurment loop
    //------------------------------------------------------------------------------------------------------------------------- 
    start = millis(); 
  
    while ((crossCount < crossings) && ((millis()-start)<timeout)) {
      wdt_reset();
      numberOfSamples++;                            //Count number of times looped.
      
      //lastSampleV=sampleV;                          //Used for digital high pass filter
      //lastSampleI=sampleI;                          //Used for digital high pass filter
     
      lastFilteredV = filteredV;                    //Used for offset removal
      lastFilteredI = filteredI;                    //Used for offset removal
  
      //-----------------------------------------------------------------------------
      // A) Read in raw voltage and current samples
      //-----------------------------------------------------------------------------
      sampleV = analogRead(V_PIN);                 //Read in raw voltage signal
      sampleI = analogRead(I_PIN);                 //Read in raw current signal
  
      //-----------------------------------------------------------------------------
      // B) Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
      //-----------------------------------------------------------------------------
      filteredV = sampleV - meanV;
      filteredI = sampleI - meanI;

      //-----------------------------------------------------------------------------
      // C) Root-mean-square method voltage
      //-----------------------------------------------------------------------------  
      sqV = filteredV * filteredV;                 //1) square voltage values
      sumSqV += sqV;                                //2) sum
      
      //-----------------------------------------------------------------------------
      // D) Root-mean-square method current
      //-----------------------------------------------------------------------------   
      sqI = filteredI * filteredI;                //1) square current values
      sumSqI += sqI;                                //2) sum 
      
      //-----------------------------------------------------------------------------
      // E) Phase calibration
      //-----------------------------------------------------------------------------
      phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);
      
      //-----------------------------------------------------------------------------
      // F) Instantaneous power calc
      //-----------------------------------------------------------------------------   
      instP = phaseShiftedV * (-1 * filteredI);          //Instantaneous Power
      sumP +=instP;                               //Sum  

      lastVCross = checkVCross;                     
      if (sampleV > startV) checkVCross = true; 
                       else checkVCross = false;
      if (numberOfSamples==1) lastVCross = checkVCross;                  
                       
      if (lastVCross != checkVCross) crossCount++;

    }
      
    double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / (ADC_COUNTS));
    double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / (ADC_COUNTS));

    Vrms = V_RATIO * sqrt(sumSqV / numberOfSamples);    
    Irms = I_RATIO * sqrt(sumSqI / numberOfSamples);
  
    //Calculation power values
    realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
    apparentPower = Vrms * Irms;
    powerFactor = realPower / apparentPower;
  
    if(realPowerAV!=0)
      realPowerAV=(realPowerAV+realPower)/2;
    else 
      realPowerAV=realPower;
      
    if(apparentPowerAV!=0)
      apparentPowerAV=(apparentPowerAV+apparentPower)/2;
    else apparentPowerAV=apparentPower;
    
    if(powerFactorAV!=0)
      powerFactorAV=(powerFactorAV+powerFactor)/2;
    else powerFactorAV=powerFactor;
       
    if(VrmsAV!=0)
      VrmsAV=(VrmsAV+Vrms)/2;
    else VrmsAV=Vrms;
    
    if(IrmsAV!=0)
      IrmsAV=(IrmsAV+Irms)/2;
    else IrmsAV=Irms;
      
    sumV = 0;
    sumI = 0;
    sumP = 0;

}
long lastMeasured = 0;
void WireDuino::loop()
{
  wdt_reset();
  if(millis()-lastMeasured>10000){
	lastMeasured = millis();
	powerMetering();
  }
}

static void Reset_AVR(){
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while(1);
}

