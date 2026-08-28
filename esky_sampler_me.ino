#define PUMP_FWD_PIN           2
#define PUMP_REV_PIN           3
#define XKC_SENSOR_PIN         4
#define HALL_SENSOR_PIN        A9
#define SPEED 13
#define SWA 67
#define BAUDRATE 9600
#define NumberOfSpins 5
#define PumpEveryXMins 1
#define DurationOfRun 12
#define ML_PER_REV 0.799
#define TARGET 500
#define CYCLES 10
#define MIN 255 // Forward Speed;
#define MAX 255
unsigned long startTime = 0;
bool isCounting = false;
bool sensor1 = false;
double MinVal,MaxVal;
double MagneticStrength;
long i,SpinCounter;
long TimeTakenToSpinMe;
double DelayTime;
double HallWakeSig;
int Purge_Rinse_Rev;
#include <avr/power.h>
#include <avr/wdt.h>
#include <MCP7940.h>
extern volatile unsigned long timer0_millis;

void setup() {
  wdt_disable();
  Serial.begin(BAUDRATE);
  pinMode(PUMP_FWD_PIN, OUTPUT);
  pinMode(PUMP_REV_PIN, OUTPUT);
  pinMode(SWA, OUTPUT);
  pinMode(SPEED, OUTPUT);
  digitalWrite(SWA, LOW);
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(PUMP_REV_PIN, LOW);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(XKC_SENSOR_PIN, INPUT);
  digitalWrite(HALL_SENSOR_PIN, LOW);
  digitalWrite(XKC_SENSOR_PIN, HIGH);
  Serial.println("GetMinsMaxs Started: ");
  GetMinsMaxs();
  Serial.println("GetMinsMaxs Ended: ");
  delay(2000);
  Serial.println("NoOfRevolutions Started: ");
  Purge_Rinse_Rev = NoOfRevolutions(); // Check this line 
  Serial.println("Let's Start this");


}

void loop() {
  int j;
  for (j=0; j < CYCLES; j++){
    Serial.println("Started the cycle.");
    int Time1 = SpinMeRev(2*Purge_Rinse_Rev);
    Serial.println("Purge");
    int Time2 = rinse();
    Serial.println("Rinse");
    int Time3 = SpinMeRev(2*Purge_Rinse_Rev);
    Serial.println("Purge");
    int Time4 = rinse();
    Serial.println("Rinse");
    int Time5 = SpinMe((int)((TARGET/CYCLES)/ML_PER_REV));
    Serial.println("Sampling Done.");
    int Time6 = SpinMeRev(Purge_Rinse_Rev*2);
    delay(80); // let the power rail settle after the pump decelerates before more serial writes
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Cycle Ended");
    Serial.print("Cycle Done: ");
    Serial.println(j+1);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.flush(); // force the buffer out fully before anything else executes
    delay(30000); // 30 second delay between cycles - no sleep for now
  }
  Serial.println("Finished Pumping");
  Serial.print("Analog Reading: ");
  Serial.println(digitalRead(XKC_SENSOR_PIN));
  Serial.print("Done Target: ");
  Serial.println(TARGET);
  delay(100000);

}
void GetMinsMaxs(){
      //this function gets the maximum and minimum values for the hall effect sensor in its current environment as the motor spins around. these values are then used to
      //determine cut off thresholds during the main program. This function is only called once, when the unit powers on.
      
      Serial.println("Calibrating peristaltic pump sensor");
      //turn motor on
      analogWrite(SPEED, MAX);
      digitalWrite(PUMP_FWD_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(250);
      //turn sensor on
      digitalWrite(HALL_SENSOR_PIN, HIGH);

      //create two values, with unrealistic numbers as thresholds.
      MinVal = 99999;
      MaxVal = 0;

      //loop around and record the minimum and highest values from the Hall effect sensor - do this for around 5seconds which is usually two or three rotations. if your pump is slow
      //you may need to incraease 5000 in the below for loop to ensure you get a good representation of values to define minimum and maximum.   
      for (i=1;i<2000;i++){
          //optain the current reading from the Hall sensor - a read of 512 (in this current program) represents no magentic field presence.
          MagneticStrength = analogRead(HALL_SENSOR_PIN)*1.0;
    

          //calculate the strength as a percentage of the number of bits possible in this current setup (1024). As such, the reading is now directionless and is a percentage; 0% indicates that 
          //the sensor is unable to measure a magentic field, while 100% indicates the field is stronger than the sensor can read
          MagneticStrength = sqrt((MagneticStrength - 512.0)*(MagneticStrength - 512.0)) / (1024)*100;
        Serial.println(MagneticStrength);
        
          //check if the current value is bigger than what we have seen previously, and if so then reset the maximum to this value, else do nothing.
          if(MagneticStrength > MaxVal){
            MaxVal = MagneticStrength;
          }
          //check if the current value is lower than what we have seen previously, and if so then reset the minimum to this value, else do nothing.
          if(MagneticStrength < MinVal){
            MinVal = MagneticStrength;
          }
          //delay for 1 millisecond
          delay(1);
      }
            Serial.println(MaxVal);
            Serial.println(MinVal);
      //turn motor off
      digitalWrite(PUMP_FWD_PIN,LOW);
      
      //turn sensor off
      digitalWrite(HALL_SENSOR_PIN,LOW);
      digitalWrite(SWA, LOW);

      //delay for 100 milliseconds
      delay(3000);
      
      //spin the device twice to initiate the sampler
      SpinMe(2);
      
      Serial.println("Finished calibrating peristaltic pump sensor");
      delay(3000);
    }

    long SpinMe(int SpinTimes){
      long StartTime;
      StartTime = millis();
      analogWrite(SPEED, MIN);
      digitalWrite(PUMP_REV_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      digitalWrite(HALL_SENSOR_PIN, HIGH);
      while (SpinCounter < SpinTimes){
        MagneticStrength = analogRead(HALL_SENSOR_PIN)*1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
        if(MagneticStrength > (MaxVal-1)){
          SpinCounter += 1;
          Serial.print(SpinCounter);
          Serial.print(": ");
          Serial.println(MagneticStrength);
        
          do{
            MagneticStrength = analogRead(HALL_SENSOR_PIN);
            MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
          }while(MagneticStrength > (MinVal+1));
        
        }
        
        delay(1);
      }
      digitalWrite(PUMP_REV_PIN, LOW);
      digitalWrite(HALL_SENSOR_PIN, LOW);
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }

    long SpinMeRev(int SpinTimes){
      long StartTime;
      StartTime = millis();
      analogWrite(SPEED, MAX);
      digitalWrite(PUMP_FWD_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(50);
      digitalWrite(HALL_SENSOR_PIN, HIGH);
      while (SpinCounter < SpinTimes){
        MagneticStrength = analogRead(HALL_SENSOR_PIN)*1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
        if(MagneticStrength > (MaxVal-1)){
          SpinCounter += 1;
          // digitalWrite(PUMP_FWD_PIN, LOW);
          // digitalWrite(SWA, LOW);
          // digitalWrite(HALL_SENSOR_PIN, LOW);
          // delay(500);
          // digitalWrite(PUMP_FWD_PIN, HIGH);
          // digitalWrite(SWA, HIGH);
          // delay(50);
          // digitalWrite(HALL_SENSOR_PIN, HIGH);

          Serial.print(SpinCounter);
          Serial.print(": ");
          Serial.println(MagneticStrength);
        
          do{
            MagneticStrength = analogRead(HALL_SENSOR_PIN);
            MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
          }while(MagneticStrength > (MinVal+1));
        
        }
        
        delay(1);
      }
      digitalWrite(PUMP_FWD_PIN, LOW);
      digitalWrite(HALL_SENSOR_PIN, LOW);
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }

int NoOfRevolutions() {
  Serial.println("Caliberation Time Baby!");
  digitalWrite(PUMP_REV_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  int RevCounter = 0;

  while (digitalRead(4) == 0) {
    MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
    MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
    
    // If the magnet is detected (strength is higher than Max threshold)
    if (MagneticStrength > (MaxVal - 1)) {
      RevCounter += 1; // Increment your revolution count
      
      // Wait in this small loop until the magnet moves away
      // This ensures one physical spin = exactly one count
      do {
        MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
      } while (MagneticStrength > (MinVal + 1));
    }
        
    delay(1);
  }
  
  // 3. SHUTDOWN AND RETURN
  digitalWrite(PUMP_REV_PIN, LOW);
  digitalWrite(HALL_SENSOR_PIN, LOW);
  digitalWrite(SWA, LOW);
  Serial.println("Caliberation Done Baby!");
  Serial.print("Rev: ");
  RevCounter += 2;
  Serial.println(RevCounter);
  delay(5000);

  return RevCounter;
}

// Bare-metal sleep replacement from Radar code
void deepSleepSecs(int32_t sec) {
  // Turn off Analog-to-Digital Converter (ADC) to save power
  ADCSRA &= ~(1 << ADEN);

  while (sec >= 4) {
    sec -= 4;
    wdt_enable(WDTO_4S);
    WDTCSR |= (1 << WDIE);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    
    // Fix millis() timing drift
    noInterrupts();
    timer0_millis += 4000;
    interrupts();
  }

  while (sec >= 1) {
    sec -= 1;
    wdt_enable(WDTO_1S);
    WDTCSR |= (1 << WDIE);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
    
    noInterrupts();
    timer0_millis += 1000;
    interrupts();
  }

  // Re-enable ADC
  ADCSRA |= (1 << ADEN);
}

// Watchdog timer interrupt service routine
ISR(WDT_vect) {
  wdt_disable();
}
int rinse(){
  long StartTime = millis();
  Serial.println("Rinsing Time Baby!");
  analogWrite(SPEED, MIN);
  digitalWrite(PUMP_REV_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  int RevCounter = 0;

  while (digitalRead(4) == 0) {
    MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
    MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
    
    // If the magnet is detected (strength is higher than Max threshold)
    if (MagneticStrength > (MaxVal - 1)) {
      RevCounter += 1; // Increment your revolution count
      
      // Wait in this small loop until the magnet moves away
      // This ensures one physical spin = exactly one count
      do {
        MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
      } while (MagneticStrength > (MinVal + 1));
    }
        
    delay(1);
  }
  
  // 3. SHUTDOWN AND RETURN
  digitalWrite(PUMP_REV_PIN, LOW);
  digitalWrite(HALL_SENSOR_PIN, LOW);
  digitalWrite(SWA, LOW);
  Serial.println("Caliberation Done Baby!");
  Serial.print("Rev: ");
  RevCounter += 2;
  Serial.println(RevCounter);
  delay(5000);
  SpinCounter = 0;
  return (millis() - StartTime);
}
long purge(int SpinTimes){
      long StartTime;
      StartTime = millis();
      analogWrite(SPEED, MAX);
      digitalWrite(PUMP_FWD_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(50);
      digitalWrite(HALL_SENSOR_PIN, HIGH);
      while (SpinCounter < SpinTimes){
        MagneticStrength = analogRead(HALL_SENSOR_PIN)*1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
        if(MagneticStrength > (MaxVal-1)){
          SpinCounter += 1;
          Serial.print(SpinCounter);
          Serial.print(": ");
          Serial.println(MagneticStrength);
        
          do{
            MagneticStrength = analogRead(HALL_SENSOR_PIN);
            MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / (1024)*100;
          }while(MagneticStrength > (MinVal+1));
        
        }
        
        delay(1);
      }
      digitalWrite(PUMP_FWD_PIN, LOW);
      digitalWrite(HALL_SENSOR_PIN, LOW);
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }
