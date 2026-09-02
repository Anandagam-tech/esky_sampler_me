#include <avr/sleep.h>    
#include <avr/power.h>    
#include <avr/wdt.h>      
#include <MCP7940.h>
#define BAUDRATE 9600
#define PUMP_REV_PIN           2
#define PUMP_FWD_PIN           3
#define SWA 67
#define SPEED 13
#define XKC_SENSOR_PIN         4
#define HALL_SENSOR_PIN        A9
#define FWD 255
#define REV 255
#define ML_PER_REV 0.799
#define TARGET 500 // In ml
#define CYCLES 10
#define RETRIES 5
#define DELAY 30000
long i = 0;
long SpinCounter = 0;
bool isCounting = false;
double MinVal,MaxVal;
double MagneticStrength;
int revolutions;
unsigned long rinseTime = 0;

bool Retries(){
  digitalWrite(SWA, LOW);
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(PUMP_REV_PIN, LOW);
  digitalWrite(HALL_SENSOR_PIN, LOW);
  digitalWrite(XKC_SENSOR_PIN, LOW);
 
  for(int i = 0; i < RETRIES; i++){
    SpinMeRev(revolutions);
    delay(5000);
    SpinMe(revolutions/4);
    delay(5000);
    SpinMeRev(revolutions);
    unsigned long anotherTime = millis();
    analogWrite(SPEED, FWD);
    digitalWrite(PUMP_FWD_PIN, HIGH);
    digitalWrite(SWA, HIGH);
    delay(25);
    digitalWrite(HALL_SENSOR_PIN, HIGH);
    digitalWrite(XKC_SENSOR_PIN, HIGH);
    int RevCounter = 0;
    while((millis() - anotherTime) < rinseTime){
      MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
      MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
      if (MagneticStrength > (MaxVal - 1)){
        RevCounter += 1;
        do{
          MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
          MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
        } while (MagneticStrength > (MinVal + 1));
      }
      delay(1);

      if(digitalRead(XKC_SENSOR_PIN) == 1){
        digitalWrite(SWA, LOW);
        digitalWrite(PUMP_FWD_PIN, LOW);
        digitalWrite(PUMP_REV_PIN, LOW);
        digitalWrite(HALL_SENSOR_PIN, LOW);
        digitalWrite(XKC_SENSOR_PIN, LOW);
        
        return true;
      }

    }
  }
  return false;
}

long Rinse(){
  unsigned long StartTime = millis();
  Serial.println("Rinsing Time Baby!");
  analogWrite(SPEED, FWD);
  digitalWrite(PUMP_FWD_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  digitalWrite(XKC_SENSOR_PIN, HIGH);
  int RevCounter = 0;

  while (digitalRead(XKC_SENSOR_PIN) == 0) {
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

    if(((millis() - StartTime) > (2 * rinseTime)) || RevCounter > (2 * revolutions)){
      bool answer = Retries();
      if (answer == true){
        StartTime = millis();
        RevCounter = 0;
      }
      else{
        Serial.print("Problem!");
        delay(10000);
        exit(0);
      }
    }
        
    delay(1);
  }
  
  // 3. SHUTDOWN AND RETURN
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(HALL_SENSOR_PIN, LOW);
  digitalWrite(SWA, LOW);
  digitalWrite(XKC_SENSOR_PIN, LOW);
  
  Serial.println("Caliberation Done Baby!");
  Serial.print("Rev: ");
  RevCounter += 2;
  Serial.println(RevCounter);
  delay(5000);
  SpinCounter = 0;
  return (millis() - StartTime);

}

int NoOfRevolutions() {
  unsigned long startTime = millis();
  Serial.println("Caliberation Time Baby!");
  analogWrite(SPEED, REV);
  digitalWrite(PUMP_REV_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  
  int RevCounter = 0;

  while (digitalRead(XKC_SENSOR_PIN) == 0) {
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
  rinseTime = millis() - startTime;
  return RevCounter;
}

long SpinMe(int SpinTimes){
      long StartTime;
      StartTime = millis();
      analogWrite(SPEED, FWD);
      digitalWrite(PUMP_FWD_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(250);
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
    long SpinMeRev(int SpinTimes){
      unsigned long StartTime;
      StartTime = millis();
      analogWrite(SPEED, REV);
      digitalWrite(PUMP_REV_PIN, HIGH);
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
      digitalWrite(PUMP_REV_PIN, LOW);
      digitalWrite(HALL_SENSOR_PIN, LOW);
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }



void GetMinsMaxs(){
  //this function gets the maximum and minimum values for the hall effect sensor in its current environment as the motor spins around. these values are then used to
  //determine cut off thresholds during the main program. This function is only called once, when the unit powers on.
    
    Serial.println("Calibrating peristaltic pump sensor");
    //turn motor on
      analogWrite(SPEED, REV);
      digitalWrite(PUMP_REV_PIN, HIGH);
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
      digitalWrite(PUMP_REV_PIN,LOW);
      
      //turn sensor off
      digitalWrite(HALL_SENSOR_PIN,LOW);
      digitalWrite(SWA, LOW);

      //delay for 100 milliseconds
      delay(100);
      
      //spin the device twice to initiate the sampler
      SpinMe(2);
      
      Serial.println("Finished calibrating peristaltic pump sensor");

}

void setup(){
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
  digitalWrite(XKC_SENSOR_PIN, LOW);
  Serial.println("GetMinsMaxs Started: ");
  GetMinsMaxs();
  Serial.println("GetMinsMaxs Ended: ");
  delay(2000);
  Serial.println("NoOfRevolutions Started: ");
  revolutions = NoOfRevolutions();
  Serial.println("NoOfRevolutions Ended: ");
  Serial.println("Loop() Started: ");
}

void loop(){
  for (int j = 0; j < CYCLES; j++){
    Serial.print("Cycle Started: ");
    SpinMeRev(2*revolutions);
    delay(5000);
    Rinse();
    delay(5000);
    SpinMeRev(2*revolutions);
    delay(5000);
    Rinse();
    SpinMe((int)((TARGET/CYCLES)/ML_PER_REV));
    SpinMeRev(2*revolutions);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Cycle Ended");
    Serial.print("Cycle Done: ");
    Serial.println(j+1);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.flush(); // force the buffer out fully before anything else executes
    delay(DELAY); // 30 second delay between cycles - no sleep for now

  }
  Serial.println("Finished Pumping");
  Serial.print("Analog Reading: ");
  Serial.println(digitalRead(XKC_SENSOR_PIN));
  Serial.print("Done Target: ");
  Serial.println(TARGET);
  exit(0);
}