#include <avr/sleep.h>    
#include <avr/power.h>    
#include <avr/wdt.h>      
#include <MCP7940.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
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
#define DELAY 60
#define SITE_DIR "microscape"
#define SITE_ID "autosampler"
#define APN "hologram"
#define MCCMNC "302220"
#define LOG_TO_WEB true                 // logging to web only occurs when true. if false SD log times may not be accurate
#define ALLOW_GSM false                 // most networks have disabled GSM
#define ALLOW_NBIOT false               // NB-IoT requires simcard support
#define ACCEPT_NON_ROAMING true         // set false to only accept roaming connections
#define LOOP_INTERVAL_MINUTES 1         // sensor scan interval (minutes)
#define AVERAGING_INTERVAL_MINUTES 6    // averaging and upload interval (minutes)
#define CONSTANT_LOGGING true           // if true, it will log to web every
#define MAX_HTTP_INTERVAL_MINUTES 60
#define HTTP_FAILS_RESET_THRESHOLD       (240) //how many HTTP fails are needed to trigger a sim7000 reset
#define TRY_POWER_ON_MAX_RESETS          (3)   //max number of reset attempts when attempting to power on the sim7000

#define POWER_ON_FAILS_RESET_LIMIT_BEGIN (6)   //how many power on resets are needed to rate limit the reset attempts
#define POWER_ON_FAILS_RESET_LIMIT_RATE  (30)  //in the rate limit mode, this is the number of reset requests needed before a reset is actually performed.

#define NET_REG_FAILS_LIMIT_BEGIN (12)   //how many power on resets are needed to rate limit the reset attempts
#define NET_REG_FAILS_LIMIT_RATE  (120)  //in the rate limit mode, this is the number of reset requests needed before a reset is actually performed.
#define MAX_OPERATORS (8)
#define POWER_ON_TRY_TIMES 5
#define HTTP_UPLOAD_TRY_TIMES 3 // to fix the issues of simcom or bad signal.
#define PWRKEY 38
#define SIM_RESET 40      // PD6
#define SIM_BUF_EN 25     // PG4
#define SIM_STATUS_BFD 45 // PC1
#define BOSL_TX 36 // TX1
#define BOSL_RX 37 // RX1
#define RTC_INTERRUPT 7
#define simCom Serial1
#define CHARBUFF 254  // SIM7000 serial response buffer,longer than 255 will cause issues
#define SIM_RESET_MIN_INTERVAL_HOURS 3
#define MIN_NON_RESET_LOOP_COUNT (SIM_RESET_MIN_INTERVAL_HOURS * 60 / LOOP_INTERVAL_MINUTES)
extern volatile unsigned long timer0_millis;

extern char response[];   // defined later in the file
uint8_t forward_speed = 255;
uint8_t reverse_speed = 255;
float ml_per_rev = 0.799;
float totalTargetVolume = 500.0f;
uint8_t cycle = 10;
uint16_t xdelay = 360;
bool state = true;
float currentML = 0;
float ml_per_sample = totalTargetVolume/cycle;
char alerts[50];
bool readState = false;
bool spinMe = true;
struct DATETIME {
public:
  void init(int16_t yr, int16_t mt, int16_t dy, int16_t hr, int16_t mn, int16_t sc) {
    this->yr = yr;
    this->mt = mt;
    this->dy = dy;
    this->hr = hr;
    this->mn = mn;
    this->sc = sc;
  }
  int16_t yr;
  int16_t mt;
  int16_t dy;
  int16_t hr;
  int16_t mn;
  int16_t sc;
} g_datetime;


long i = 0;
long SpinCounter = 0;
bool isCounting = false;
double MinVal,MaxVal;
double MagneticStrength;
int revolutions;
unsigned long rinseTime = 0;
const unsigned long SERVER_CHECK_INTERVAL = 360000;
unsigned long lastServerCheck = 0;
const uint16_t MAX_STATE_SEC = 300;
int lost_seconds = 0;
char CBC[5];
char CSQ[5];
struct Operator {
    uint8_t status;
    char mccmnc[7];
    uint8_t netact;
    uint8_t rssi;
};
int isRegistered(const uint32_t timeout_ms = 591);
int netSelect(const char* mccmnc);
int getMccmnc(char* mccmnc, size_t mccmnc_size);
int netSearch(void);
bool sendATcmd(String ATcommand, const char *expctAns, uint32_t timeout, int8_t tries = 5);
bool sendAlert(String message);
bool sendAlertStandalone(String message);
bool serialBegin();
void serialEnd();
bool tryPowerOnSimCom(int8_t tries = 5);
bool tryPowerOffSimCom();
void openbearer();
void closeBearer();
void netUnreg();
bool hardResetSimCom();
void deepSleepSecs(int32_t seconds);
bool ReadAll();
bool ReadState();
bool waitForHttpAction(uint32_t timeout_ms);
bool netReg(void);
bool isSimComOn();
bool hardPowerOffSimCom();
void CBCread();
void CSQread();
bool pullNetTime();
void updateRTC(const DATETIME*);
void xDelay(uint32_t ms);
bool Rinse();
bool RinseOnce();
bool Retries();
int NoOfRevolutions();
long SpinMe(int);
long SpinMeRev(int);
void GetMinsMaxs();
void initHardware();
void stopAllHardware();
bool negoSimComBaudRate(int32_t baud);
void _dbgPrintDateTime(const DATETIME& datetime);
void storeCBCresponse();
int storeOperators();
int compare_rssi(const void *a, const void *b);
void datetime2y2k(const DATETIME* cal, const int16_t tz4, uint32_t* y2ksecs);
void y2k2datetime(DATETIME* cal, const uint32_t y2ksecs);
int getRSSI(uint8_t* rssi_p);
int _dbgPrintOperators(void);
bool powerOnSimCom();
// === FIX: new helper for zombie-modem recovery ===
void modemHardPowerCycle();
uint8_t operators_len = 0;
Operator operators[MAX_OPERATORS];
int getRSSI(uint8_t* rssi_p){
    int ret = 0;
    int rssi_temp = 0;
    sendATcmd(F("AT+CSQ"), "OK", 1000, 1);
    char* start = strchr(response, '+');
    if(start == NULL){return false;}

    ret = sscanf(start, "+CSQ: %d,%*d", &rssi_temp);
    if(ret == 1){
      *rssi_p = (uint8_t)rssi_temp;
      return true;
    }
    *rssi_p = 99;
    return false;
}


void datetime2y2k(const DATETIME* cal, const int16_t tz4, uint32_t* y2ksecs){
    struct tm t;
    t.tm_sec = cal->sc; 
    t.tm_min = cal->mn; 
    t.tm_hour = cal->hr; 
    t.tm_mday = cal->dy; 
    t.tm_mon = (cal->mt -1); 
    t.tm_year = cal->yr + 100; 
    t.tm_isdst = 0;

    *y2ksecs = mktime(&t);
    *y2ksecs -= 900*(uint32_t)tz4;//we need to subtract the timezone given in quarter hour increments (900 seconds)

}

void y2k2datetime(DATETIME* cal, const uint32_t y2ksecs){
  uint32_t seconds, minutes, hours, days, year, month;
  uint32_t day_of_week;
  seconds = y2ksecs;

  minutes  = seconds / 60;
  hours    = minutes / 60;
  days     = hours   / 24;

  seconds = seconds % 60;
  minutes = minutes % 60;
  hours   = hours   % 24;

  /* avr time starts in 2000 */
  year    = 2000;

  while(1)
  {
    int     leap_year   = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    uint16_t days_in_year = leap_year ? 366 : 365;
    if (days >= days_in_year)
    {
      days      -= days_in_year;
      ++year;
    }
    else
    {
      /* calculate the month and day */
      static const uint8_t days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      for(month = 0; month < 12; ++month)
      {
        uint8_t dim = days_in_month[month];

        /* add a day to feburary if this is a leap year */
        if (month == 1 && leap_year)
          ++dim;

        if (days >= dim)
          days -= dim;
        else
          break;
      }
      break;
    }
  }

  cal->sc  = seconds;
  cal->mn  = minutes;
  cal->hr = hours;
  cal->dy = days + 1;
  cal->mt  = month + 1;
  cal->yr = year - 2000;//for correct calender epoch
}



ISR(WDT_vect) {
  // This function runs automatically when the watchdog timer expires.
  // Its ONLY job is to prevent a system reset and wake the CPU.
  wdt_disable();
}
MCP7940_Class MCP7940;
// SIM7000
char response[CHARBUFF]; // sim7000 serial response buffer
String dataStr;          // Transmit URL
bool retrieveFromWeb() {
  bool ret = false;
  static uint16_t upload_fails = 0;
  static uint16_t power_on_fails = 0;
  static uint16_t net_reg_fails = 0;

  Serial.print(F("upload fails: "));
  Serial.print(upload_fails);
  Serial.print(F(" power_on_fails: "));
  Serial.print(power_on_fails);
  Serial.print(F(" net_reg_fails: "));
  Serial.println(net_reg_fails);

  if (net_reg_fails > NET_REG_FAILS_LIMIT_BEGIN && 
      (net_reg_fails % NET_REG_FAILS_LIMIT_RATE)) {
    Serial.println(F("netReg failure high. Limiting logging"));
    net_reg_fails++;
    return false;
  }

  if (net_reg_fails > NET_REG_FAILS_LIMIT_BEGIN &&
     !(net_reg_fails % 3 * NET_REG_FAILS_LIMIT_RATE)) {
    Serial.println(F("netReg failure high. Reseting"));
    hardResetSimCom();
  }
  if (isSimComOn()){
    Serial.println(F("Modem unexpectedly ON at cycle start - forcing clean power cycle"));
    modemHardPowerCycle();
  }
  Serial.println(F("Initialising SIM 7000"));
  simCom.begin(BAUDRATE);

  uint8_t reset = false;
  for (uint8_t i = 0; i < TRY_POWER_ON_MAX_RESETS; i++) {
    if (reset == true) {
      reset = false; 
      power_on_fails++;
      if (power_on_fails < POWER_ON_FAILS_RESET_LIMIT_BEGIN || 
           !(power_on_fails % POWER_ON_FAILS_RESET_LIMIT_RATE)) {
          Serial.println(F("cannot communicate with simCom. resetting"));
          hardResetSimCom();
      }
    }
    
    ret = tryPowerOnSimCom(POWER_ON_TRY_TIMES);
    if(ret == false){reset = true; continue;}

    ret = serialBegin();
    if(ret == false){reset = true; continue;}
    
    break;
  }
  if(ret == false){goto cleanup;}

  ret = netReg();
  if (ret == false) {
    net_reg_fails++;
    goto net_reg_cleanup;
  } else {
    net_reg_fails = 0;
  }

  openbearer();
  CBCread();
  CSQread();

  if (pullNetTime()) {
    Serial.print("Updating to UTC time: ");
    _dbgPrintDateTime(g_datetime);
    updateRTC(&g_datetime);
  }

  for (uint8_t i = 0; i < HTTP_UPLOAD_TRY_TIMES; i++) {
    ret = ReadAll();
    if (ret == false) {
      upload_fails++;
      if(strstr(response, "+HTTPACTION: 0,601") != NULL || strstr(response, "DEACT") != NULL){
        Serial.println(F("Bearer died mid-request - reopening"));
        sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
        closeBearer();
        delay(1000);
        openbearer();
        delay(500);
      }
      if (upload_fails > HTTP_FAILS_RESET_THRESHOLD) {
        upload_fails = 0;
        Serial.println(F("HTTP failure rate high. resetting"));
        hardResetSimCom();
        goto net_reg_cleanup;
      }
      continue;
    }
    break;
  }

  closeBearer();

net_reg_cleanup:
  netUnreg();

cleanup:
  serialEnd();
  tryPowerOffSimCom();
  Serial.print(F("LogToWeb() ret: "));
  Serial.println(ret);
  return ret;
}


bool ReadAll() {
  bool condition = true;
  if(!ReadState()){
    return false;
  }
  xDelay(500);
  if(!ReadMLSample()){
    sendAlert("500");
    return false;
  }
  xDelay(500);
  if(!ReadMLRev()){
    return false;
  }

  xDelay(500);
  if(!ReadCycle()){
    return false;
  }
  xDelay(500);
  if(!ReadTotalTargetVolume()){
    return false;
  }
  xDelay(500);
  if(!ReadXDelay()){
    return false;
  }
  return true;

  
}

bool ReadState(){
  bool ret = false;

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "state";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    readState = false;
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    readState = false;
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    readState = false;
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    readState = false;
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    readState = false;
    return false;
  }
  // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);

  char saved_response[CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';

  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) {
    readState = false;
    return false;
    }

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) {
    readState = false;
    return false;
    }
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    readState = false;
    return false;
  }
  uint8_t tempState = 0;
  int parsed = sscanf(p, "%hhu", &tempState);
  if (parsed != 1){
    Serial.println(F("State Not Found! "));
    readState = false;
    return false;
  }
  state = (tempState!= 0);
  readState = true;
  return true;

}
bool ReadMLSample(){
  bool ret = false;

  if (readState == false){
    return ret;
  }

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "ml_per_sample";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }  // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);
  char saved_response[CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';
  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) return false;

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) return false;
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    return false;
  }
  char buffer[15];
  int parsed = sscanf(p, "%[^\r\n]", buffer);
  if (parsed != 1){
    Serial.println(F("ml_per_sample Not Found! "));
    return false;
  }
  ml_per_sample = atof(buffer);
  return true;

}
bool ReadMLRev(){
  bool ret = false;

  if (readState == false){
    return ret;
  }

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "ml_per_rev";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }
    // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);
  char saved_response[CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';
  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) return false;

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) return false;
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    return false;
  }
  char buffer[15];
  int parsed = sscanf(p, "%[^\r\n]", buffer);
  if (parsed != 1){
    Serial.println(F("ml_per_rev Not Found! "));
    return false;
  }
  ml_per_rev = atof(buffer);
  return true;

}
bool ReadCycle(){
  bool ret = false;

  if (readState == false){
    return ret;
  }

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "cycle";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }
  // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);
  char saved_response[CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';
  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) return false;

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) return false;
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    return false;
  }
  int parsed = sscanf(p, "%hhu", &cycle);
  if (parsed != 1){
    Serial.println(F("cycle Not Found! "));
    return false;
  }
  return true;

}

bool ReadTotalTargetVolume(){
  bool ret = false;

  if (readState == false){
    return ret;
  }

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "totalTargetVolume";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }
  // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);
  char saved_response [CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';

  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) return false;

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) return false;
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    return false;
  }
  char buffer[15];
  int parsed = sscanf(p, "%[^\r\n]", buffer);
  if (parsed != 1){
    Serial.println(F("cycle Not Found! "));
    return false;
  }
  totalTargetVolume = atof(buffer);
  return true;

}

bool ReadXDelay(){
  bool ret = false;

  if (readState == false){
    return ret;
  }

  // 1. Build URL with &Key=
  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/ReadMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv&Key=";
  dataStr += "xdelay";
  dataStr += "\"";

  // 2. Initialize HTTP session
  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 3. Set URL
  sendATcmd(dataStr, "OK", 2000);
  if (strstr(response, "ERROR")) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 4. Trigger GET request (0 = GET)
  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }
  // 6. Read the payload
  sendATcmd(F("AT+HTTPREAD"), "OK", 2000);
  char saved_response [CHARBUFF];
  strncpy(saved_response, response, CHARBUFF - 1);
  saved_response[CHARBUFF - 1] = '\0';
  // 7. Cleanup HTTP session immediately
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);

  // 8. Extract the single value from response buffer
  // Response looks like: "+HTTPREAD: 3\r\n500\r\nOK"
  char *p = strstr(saved_response, "+HTTPREAD:");
  if (p == NULL) return false;

  p = strstr(p, "\r\n"); // Find end of +HTTPREAD line
  if (p == NULL) return false;
  p += 2;                // Skip \r\n (p now points to the actual data)

  if (strstr(p,"error") != NULL || strlen(p) == 0){
    Serial.print(F("Key not found!"));
    return false;
  }
  char buffer[15];
  int parsed = sscanf(p, "%hu", &xdelay);
  if (parsed != 1){
    Serial.println(F("xDelay Not Found! "));
    return false;
  }
  return true;

}


// bool Parse(){
//   if (!ReadAll()){
//     return false; 
//   }

//   char *p = strstr(response, "+HTTPREAD:");
//   if (p == NULL){
//     return false;
//   }
//   Serial.println(p);
//   p = strstr(p, "\r\n");
//   if (p == NULL){
//     return false;
//   }
//   p += 2;

//   char dummyTime[25];
//   char mlStr[15];
//   char ml_sample[15];
//   char current[15];
//   uint8_t tempState = 0;
//   int parsedCount = sscanf(p, "%[^,],%hhu,%[^,],%[^,],%hu,%hhu,%hu,%[^,],%[^\n]",dummyTime, &tempState, ml_sample, mlStr, &totalTargetVolume, &cycle, &xdelay, current,alerts);
//   if (parsedCount != 8){
//     Serial.println("Not all variables were parsed! ");
//     return false;
//   }
//   ml_per_rev = atof(mlStr);
//   ml_per_sample = atof(ml_sample);
//   currentML = atof(current);
//   state = (tempState!= 0);
//   return true;

// }
bool serialBegin() {
  pinMode(BOSL_RX, OUTPUT);
  digitalWrite(BOSL_RX, HIGH);
  pinMode(BOSL_TX, INPUT_PULLUP);
  simCom.begin(BAUDRATE);
  xDelay(1000);
  return negoSimComBaudRate(BAUDRATE);
}

bool negoSimComBaudRate(int32_t baud) {
  Serial.println(F("\n== nego baud rate =="));
  simCom.begin(baud);
  if (sendATcmd(F("AT"), "OK", 1000, 16) != 0) {
    Serial.println(F("\nSim7000 current baud rate is 9600 :)"));
    return sendATcmd(F("ATE0"), "OK", 1000, 4);
  }
  return false;
}

void serialEnd() {
  while (simCom.available()) {
    simCom.read();
  }
  simCom.flush();
  xDelay(300);
  simCom.end();
  // pull down RX TX to save power
  pinMode(BOSL_RX, OUTPUT);
  pinMode(BOSL_TX, OUTPUT);
  digitalWrite(BOSL_RX, LOW);
  digitalWrite(BOSL_TX, LOW);
}

bool isSimComOn() { return digitalRead(SIM_STATUS_BFD); }

bool powerOnSimCom() {
  Serial.println(F("\nPowerOnSimCom"));
  delay(50);
  digitalWrite(PWRKEY, LOW);
  xDelay(1000);
  digitalWrite(PWRKEY, HIGH);
  xDelay(4000);
  for (int8_t i = 0; i < 10; i++) {
    if (isSimComOn()) {
      return true;
    }
    xDelay(500);
  }
  return false;
}

// bool tryPowerOnSimCom(int8_t tries = 5) {
//   Serial.println(F("\nTry to power on SimCom:"));
//   if (isSimComOn()) {
//     Serial.println(F("\nSimCom is already on"));
//     Serial.println(F("succeed."));
//     return true;
//   }

//   for (int8_t i = 0; i < 3; i++) {
//     if (powerOnSimCom()) {
//       Serial.println(F("succeed."));
//       return true;
//     }
//   }

//   return false;
// }
// === FIX: verify the UART is actually alive before trusting SIM_STATUS_BFD.
// After a brownout, the modem can report STATUS=HIGH while its UART is dead,
// causing the code to skip re-powering forever. Also use the `tries` param
// instead of hardcoded 3.
bool tryPowerOnSimCom(int8_t tries = 5) {
  Serial.println(F("\nTry to power on SimCom:"));
  if (isSimComOn()) {
    simCom.begin(BAUDRATE);
    xDelay(200);
    while (simCom.available()) simCom.read();
    simCom.println(F("AT"));

    char probe[32] = {0};
    uint8_t idx = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 1000){
      if(simCom.available()){
        char c = simCom.read();
        if (idx < sizeof(probe) - 1) probe[idx++] = c;
      }
    }
    while (simCom.available()) simCom.read();
    bool uartAlive = (strstr(probe, "OK") != NULL) || (strstr(probe, "AT") != NULL);
    if (uartAlive) {
      Serial.println(F("\nSimCom is already on"));
      Serial.println(F("succeed."));
      return true;
    }
    Serial.println(F("SimCom reports ON but UART is dead - forcing power cycle"));
    modemHardPowerCycle();
  }

  for (int8_t i = 0; i < tries; i++) {
    if (powerOnSimCom()) {
      Serial.println(F("succeed."));
      return true;
    }
  }
  return false;
}

bool tryPowerOffSimCom() {
  Serial.println(F("\nTry to power off SimCom:"));
  if (isSimComOn() == false) {
    return true;
  }

  for (int8_t i = 0; i < 3; i++) {
    if (hardPowerOffSimCom()) {
      Serial.println(F("Power off Sim succeed."));
      return true;
    }
  }
  return false;
}

bool hardPowerOffSimCom() {
  Serial.println(F("\nhardPowerOffSimCom"));
  delay(50);
  digitalWrite(PWRKEY, LOW);
  xDelay(1300);
  digitalWrite(PWRKEY, HIGH);
  int8_t i = 0;
  for (; i < 20; i++) {
    if (isSimComOn() == false) {
      return true;
    }
    xDelay(500);
  }
  return false;
}

bool hardResetSimCom() {
  Serial.println(F("Reset SIM7000"));
  pinMode(SIM_RESET, OUTPUT);
  digitalWrite(SIM_RESET, LOW);
  xDelay(2000);
  digitalWrite(SIM_RESET, HIGH);
  xDelay(4000);
  for (int8_t i = 0; i < 10; i++) {
    if (isSimComOn()) {
      return true;
    }
    xDelay(500);
  }
  return false;
}

// void openbearer() {
//   // set CSTT - if it is already set, then no need to do again...
//   sendATcmd(F("AT+CSTT?"), "OK", 1000);
//   if (strstr(response, APN) != NULL) {
//     // this means the cstt has been set, so no need to set again!
//     Serial.println(F("CSTT already set to APN ...no need to set again"));
//   } else {
//     sendATcmd(F("AT+CSTT=\"" APN "\""), "OK", 1000);
//   }
//   // close open bearer
//   sendATcmd(F("AT+SAPBR=2,1"), "OK", 1000);
//   if (strstr(response, "1,1") == NULL) {
//     if (strstr(response, "1,3") == NULL) {
//       sendATcmd(F("AT+SAPBR=0,1"), "OK", 1000);
//     }
//     sendATcmd(F("AT+SAPBR=3,1,\"APN\",\"" APN "\""), "OK", 1000);  // set bearer apn
//     sendATcmd(F("AT+SAPBR=1,1"), "OK", 1000);
//   }
// }

// === FIX: old openbearer() fired SAPBR=1,1 while the modem was still
// in state 1,3 (deactivating), which raced the internal state machine.
// That's the direct cause of "+SAPBR 1: DEACT" mid-request and the
// "+HTTPACTION: 0,601,0" errors. Wait for a stable state, then activate,
// then wait for a real IP before returning.
void openbearer() {
  sendATcmd(F("AT+CSTT?"), "OK", 1000);
  if (strstr(response, APN) != NULL) {
    Serial.println(F("CSTT already set to APN ...no need to set again"));
  } else {
    sendATcmd(F("AT+CSTT=\"" APN "\""), "OK", 1000);
  }

  // Wait for bearer to reach a settled state (1,1 open, or 1,0 closed)
  bool ready = false;
  for (int i = 0; i < 20; i++) {
    sendATcmd(F("AT+SAPBR=2,1"), "OK", 1000);
    if (strstr(response, "1,1") != NULL) {
      Serial.println(F("Bearer already open"));
      return;
    }
    if (strstr(response, "1,0") != NULL) { ready = true; break; }
    Serial.print(F("Bearer transitioning, wait "));
    Serial.println(i);
    delay(500);
  }
  if (!ready) {
    Serial.println(F("Forcing SAPBR=0,1 to clear stuck state"));
    sendATcmd(F("AT+SAPBR=0,1"), "OK", 5000);
    delay(1500);
  }

  sendATcmd(F("AT+SAPBR=3,1,\"APN\",\"" APN "\""), "OK", 1000);
  sendATcmd(F("AT+SAPBR=1,1"), "OK", 5000);

  // Wait for it to actually open with a real IP, not 0.0.0.0
  for (int i = 0; i < 30; i++) {
    sendATcmd(F("AT+SAPBR=2,1"), "OK", 1000);
    if (strstr(response, "1,1") != NULL && strstr(response, "0.0.0.0") == NULL) {
      Serial.println(F("Bearer opened with valid IP"));
      return;
    }
    Serial.print(F("Waiting for IP ("));
    Serial.print(i); Serial.println(F("/30)"));
    delay(500);
  }
  Serial.println(F("WARNING: bearer did not open with valid IP"));
}

// === FIX: new helper to fully power-cycle the modem when it's in a zombie
// state (STATUS pin HIGH, UART dead). Pulls PWRKEY low long enough to force
// emergency shutdown, waits for the rail to discharge, then powers on.
void modemHardPowerCycle() {
  Serial.println(F("modemHardPowerCycle: forcing shutdown"));
  digitalWrite(PWRKEY, LOW);
  xDelay(1500);
  digitalWrite(PWRKEY, HIGH);
  xDelay(5000);      // wait for modem rail to fully discharge
  Serial.println(F("modemHardPowerCycle: powering back on"));
  powerOnSimCom();
}

void closeBearer() { sendATcmd(F("AT+SAPBR=0,1"), "OK", 1000); }

bool sendATcmd(String ATcommand, const char *expctAns, uint32_t timeout,
               int8_t tries) {
  uint32_t timeStart;
  bool answer;
  uint8_t a = 0;

  do {
    a++;
    Serial.println();
    Serial.println(ATcommand);

    answer = false;
    timeStart = 0;
    xDelay(100);

    while (simCom.available() > 0) {
      simCom.read(); // Clean the input buffer
    }

    simCom.println(ATcommand); // Send the AT command

    uint8_t i = 0;
    timeStart = millis();
    memset(response, '\0', CHARBUFF); // Initialize the string

    // this loop waits for the answer
    do {
      if (simCom.available() != 0) {
        if(i < CHARBUFF - 1){
          response[i++] = simCom.read();
          response[i] = '\0';
        } else{
          simCom.read();
        }
        // check if the desired answer is in the response of the module
        if (strstr(response, expctAns) != NULL) {
          answer = true;
        }
      }
      // Waits for the asnwer with time out
    } while ((answer == false) && ((millis() - timeStart) < timeout));
    response[CHARBUFF - 1] = '\0';

    Serial.print(F("response:"));
    Serial.println(response);

    if (!expctAns || expctAns[0] == '\0') {
      answer = true;
    }

  } while (answer == false && a < tries);

  a = 0;
  return answer;
}
bool sendAlertStandalone(String message) {
  bool ret = false;

  Serial.println(F("sendAlertStandalone: bringing network up"));

  simCom.begin(BAUDRATE);

  ret = tryPowerOnSimCom(POWER_ON_TRY_TIMES);
  if (!ret) { Serial.println(F("sendAlertStandalone: power-on failed")); goto cleanup; }

  ret = serialBegin();
  if (!ret) { Serial.println(F("sendAlertStandalone: serialBegin failed")); goto cleanup; }

  ret = netReg();
  if (!ret) { Serial.println(F("sendAlertStandalone: netReg failed")); goto cleanup; }

  openbearer();
  ret = sendAlert(message);
  closeBearer();
  netUnreg();

cleanup:
  serialEnd();
  tryPowerOffSimCom();
  Serial.print(F("sendAlertStandalone ret: "));
  Serial.println(ret);
  return ret;
}

bool sendAlert(String message){

  bool ret = false;

  dataStr = "AT+HTTPPARA=\"URL\",\"http://www.bosl.com.au/IoT/";
  dataStr += SITE_DIR;
  dataStr += "/scripts/WriteMe_v2.php?SiteName=";
  dataStr += SITE_ID;
  dataStr += ".csv";
  dataStr += "&currentML=";
  dataStr += currentML;
  dataStr += "&alerts=";
  strncpy(alerts, message.c_str(), sizeof(alerts)-1);
  alerts[sizeof(alerts)-1] = '\0';
  dataStr += alerts;

  // dataStr += "&CBC=";
  // dataStr += CBC;
  // dataStr += "&ANGLE=";
  // dataStr += ANGLE;
  // dataStr += "&VEL=";
  // dataStr += VEL;
  // dataStr += "&PEAKS=";
  // dataStr += PEAKS;
  // dataStr += "&DEPTH=";
  // dataStr += FDIST;
  // dataStr += "&AMP=";
  // dataStr += AMP;
  // dataStr += "&CSQ=";
  // dataStr += CSQ;
  // dataStr += "&AVEL=";
  // dataStr += AVEL;
  // // dataStr += "&AFDIST=";
  // // dataStr += AFDIST;
  dataStr += "\"";

  sendATcmd(F("AT+HTTPINIT"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }
  sendATcmd(F("AT+HTTPPARA=\"CID\",1"), "OK", 1000);
  if (strstr(response, "ERROR")) {
    return false;
  }

  sendATcmd(dataStr, "OK", 2000);  // check what the HTTPTIMEOUT is

  if (sendATcmd(F("AT+HTTPACTION=0"), "OK", 5000) == false) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 5. Wait for the +HTTPACTION URC
  if (!waitForHttpAction(15000)) {
    sendATcmd(F("AT+HTTPTERM"), "OK", 1000);
    return false;
  }

  // 6. Wait for the session to become idle, then close it
  sendATcmd(F("AT+HTTPSTATUS?"), "+HTTPSTATUS: GET,0,0,0", 1000, 2);
  sendATcmd(F("AT+HTTPTERM"), "OK", 1000, 3);
  return true;
}

// bool parseCheck(){
//   stopAllHardware();
//   Parse();
//   while(state == false){
//     for(int i = 0; i < RETRIES; i++){
//       deepSleepSecs(300);
//       lost_seconds += 300;
//       if(state == true){
//         break;
//       }
//     }
//     if(state == true){
//       break;
//     }
//     sendAlert("Error!");
//   }
//   return true;

// }

void deepSleepSecs(int32_t seconds) {
  // 1. Turn off the ADC (Analog-to-Digital Converter) to save power
  //    You won't be reading sensors while sleeping anyway
  ADCSRA &= ~(1 << ADEN);

  // 2. Sleep in 8-second chunks (maximum single watchdog interval)
  while (seconds >= 8) {
    seconds -= 8;

    // Arm the watchdog timer for 8 seconds in INTERRUPT mode
    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE);  // Enable interrupt mode (not reset mode)

    // Configure and enter deepest sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();  // <-- CPU STOPS HERE

    // --- CPU wakes up here after 8 seconds ---
    sleep_disable();
    wdt_disable();

    // Manually add 8000ms to the Arduino's clock so millis() stays accurate
    cli();
    timer0_millis += 8000;
    sei();
  }

  // 3. Sleep in 4-second chunks for remaining time
  while (seconds >= 4) {
    seconds -= 4;

    wdt_enable(WDTO_4S);
    WDTCSR |= (1 << WDIE);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();

    sleep_disable();
    wdt_disable();

    cli();
    timer0_millis += 4000;
    sei();
  }

  // 4. Sleep in 1-second chunks for any leftover seconds
  while (seconds >= 1) {
    seconds -= 1;

    wdt_enable(WDTO_1S);
    WDTCSR |= (1 << WDIE);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();

    sleep_disable();
    wdt_disable();

    cli();
    timer0_millis += 1000;
    sei();
  }

  // 5. Re-enable the ADC so analogRead() works again after waking
  ADCSRA |= (1 << ADEN);
}

void xDelay(uint32_t ms) {
  // Flush serial so messages don't get cut off
  if (Serial) {
    Serial.flush();
    if (ms >= 100) delay(20);
  }

  // Calculate how many 64ms slices we can fit
  uint32_t slices = ms / 64;

  // Slow the CPU clock down by 64x (16MHz -> 250kHz)
  // This reduces power consumption dramatically during short waits
  clock_prescale_set(clock_div_64);

  // Wait for the reduced time (because clock is 64x slower, 
  // delay(1) actually takes 64ms of real time)
  delay(slices);

  // Restore CPU clock to full speed
  clock_prescale_set(clock_div_1);

  // Manually fix the Arduino's internal clock (millis)
  // because slowing the CPU also slowed down millis()
  cli();  // Disable interrupts briefly to safely edit the timer
  timer0_millis += 63 * slices;  // Add back the "lost" time
  sei();  // Re-enable interrupts

  // Handle any remaining milliseconds that didn't fit into 64ms slices
  delay(ms - 64 * slices);
}
int _dbgPrintOperators(void) {
  Serial.print(F("Operators Length: "));
  Serial.println(operators_len);

  for (int i = 0; i < operators_len; ++i) {
    Serial.print(F("Operator "));
    Serial.print(i + 1);
    Serial.print(F(": Status - "));
    Serial.print(operators[i].status);
    Serial.print(F(", MCCMNC - "));
    Serial.print(operators[i].mccmnc);
    Serial.print(F(", Netact - "));
    Serial.print(operators[i].netact);
    Serial.print(F(", RSSI - "));
    Serial.println(operators[i].rssi);
  }
  return true;
}

int storeOperators(){
    char* start = NULL;
    char* end = NULL;
    size_t len = 0;

    operators_len = 0;

    start = strstr(response, "+COPS: ");//make sure we are reading the +COPS line
    if(start == NULL){return false;}

    for(uint8_t i = 0; i < MAX_OPERATORS; i++){
        if(start == strstr(start, ",,")){return true;} //we are at the end of the list

        start = strstr(start, "(");//start of a operators listing
        if(start == NULL){return false;}
        end   = strstr(start, "),") + 1;//end of an operator listing
        if(end == NULL){return false;}

        len = end - start;//check that the matches are of reasonable length
        if(len > CHARBUFF){return false;} 

        Operator* o = &operators[operators_len];
        uint8_t matches = sscanf(start, "(%d,%*[^,],%*[^,],\"%6[^\"]\",%d)", &o->status, &o->mccmnc, &o->netact);
        if(matches == 3){
            o->rssi = 99;//not known or not detectable
            operators_len++;
        } 
        else {return false;}

        start = end;
    }
    return false;
}

int compare_rssi(const void *a, const void *b){
    int16_t rssi_a = ((Operator*)a)->rssi;
    int16_t rssi_b = ((Operator*)b)->rssi;

    if(rssi_a == 99 && rssi_b == 99){return 0;}
    if(rssi_a == 99){return 1;}
    if(rssi_b == 99){return -1;}

    return  rssi_a > rssi_b ? -1 : 
           (rssi_b > rssi_a ?  1 : 0);

}


void initHardware(){
  pinMode(PUMP_FWD_PIN, OUTPUT);
  pinMode(PUMP_REV_PIN, OUTPUT);
  pinMode(SWA, OUTPUT);
  pinMode(SPEED, OUTPUT);
  digitalWrite(SWA, LOW);
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(PUMP_REV_PIN, LOW);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(XKC_SENSOR_PIN, INPUT_PULLUP);   // instead of INPUT + digitalWrite(LOW)
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, HIGH);
  pinMode(SIM_RESET, OUTPUT);
  digitalWrite(SIM_RESET, HIGH);
  pinMode(SIM_STATUS_BFD, INPUT); 
  pinMode(SIM_BUF_EN, OUTPUT);
  digitalWrite(SIM_BUF_EN, HIGH); // or LOW, depending on hardware

}
void stopAllHardware(){
  digitalWrite(SWA, LOW);
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(PUMP_REV_PIN, LOW);
  digitalWrite(XKC_SENSOR_PIN, LOW);
}
bool Retries(){
  digitalWrite(SWA, LOW);
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(PUMP_REV_PIN, LOW);
  digitalWrite(XKC_SENSOR_PIN, LOW);
 
  for(int i = 0; i < RETRIES; i++){
    SpinMeRev(revolutions);
    xDelay(5);
    SpinMe(revolutions/4);
    xDelay(5);
    SpinMeRev(revolutions);
    unsigned long anotherTime = millis();
    analogWrite(SPEED, forward_speed);
    digitalWrite(PUMP_FWD_PIN, HIGH);
    digitalWrite(SWA, HIGH);
    delay(25);
    digitalWrite(XKC_SENSOR_PIN, HIGH);
    lost_seconds += 10;
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
        digitalWrite(XKC_SENSOR_PIN, LOW);
        
        return true;
      }

    }
  }
  return false;
}

// One rinse attempt. Returns true if XKC sensor triggered in time,
// false if the rinse timed out or span too many revolutions.
bool RinseOnce() {
  unsigned long StartTime = millis();
  Serial.println(F("Rinsing Time Baby!"));
  analogWrite(SPEED, forward_speed);
  digitalWrite(PUMP_FWD_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  digitalWrite(XKC_SENSOR_PIN, HIGH);

  int RevCounter = 0;

  while (digitalRead(XKC_SENSOR_PIN) == 0) {
    MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
    MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;

    if (MagneticStrength > (MaxVal - 1)) {
      RevCounter += 1;
      do {
        MagneticStrength = analogRead(HALL_SENSOR_PIN) * 1.0;
        MagneticStrength = sqrt((MagneticStrength - 512.0) * (MagneticStrength - 512.0)) / 1024.0 * 100.0;
      } while (MagneticStrength > (MinVal + 1));
    }

    if (((millis() - StartTime) > (2UL * rinseTime)) || RevCounter > (2 * revolutions)) {
      // Failed: too slow or too many revolutions
      digitalWrite(PUMP_FWD_PIN, LOW);
      digitalWrite(SWA, LOW);
      digitalWrite(XKC_SENSOR_PIN, LOW);
      Serial.println(F("RinseOnce: FAILED (timeout/excess revs)"));
      return false;
    }

    delay(1);
  }

  // Success
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(SWA, LOW);
  digitalWrite(XKC_SENSOR_PIN, LOW);
  Serial.println(F("RinseOnce: OK"));
  return true;
}

// Robust rinse: try once, then 3 retries with clearing spins in between.
bool Rinse() {
  if (RinseOnce()) return true;

  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print(F("Rinse retry "));
    Serial.println(attempt);

    // Clear the intake tube before retrying
    SpinMeRev(revolutions);
    xDelay(5);
    SpinMe(revolutions / 4);
    xDelay(5);

    if (RinseOnce()) return true;
  }

  Serial.println(F("Rinse: all retries failed"));
  return false;
}
int NoOfRevolutions() {
  unsigned long startTime = millis();
  Serial.println("Caliberation Time Baby!");
  analogWrite(SPEED, forward_speed);
  digitalWrite(PUMP_FWD_PIN, HIGH);
  digitalWrite(SWA, HIGH);
  
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
  digitalWrite(PUMP_FWD_PIN, LOW);
  digitalWrite(SWA, LOW);
  Serial.println("Caliberation Done Baby!");
  Serial.print("Rev: ");
  RevCounter += 2;
  Serial.println(RevCounter);
  xDelay(3);
  rinseTime = millis() - startTime;
  return RevCounter;
}

long SpinMe(int SpinTimes){
      unsigned long StartTime;
      StartTime = millis();
      analogWrite(SPEED, forward_speed);
      digitalWrite(PUMP_FWD_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(250);
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
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }
    long SpinMeRev(int SpinTimes){
      unsigned long StartTime;
      StartTime = millis();
      analogWrite(SPEED, reverse_speed);
      digitalWrite(PUMP_REV_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(50);
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
          //

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
      digitalWrite(SWA, LOW);

      SpinCounter = 0;
      return (millis() - StartTime);
    }



void GetMinsMaxs(){
  //this function gets the maximum and minimum values for the hall effect sensor in its current environment as the motor spins around. these values are then used to
  //determine cut off thresholds during the main program. This function is only called once, when the unit powers on.
    
    Serial.println("Calibrating peristaltic pump sensor");
    //turn motor on
      analogWrite(SPEED, reverse_speed);
      digitalWrite(PUMP_REV_PIN, HIGH);
      digitalWrite(SWA, HIGH);
      delay(250);
    

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
      
      
      digitalWrite(SWA, LOW);

      //delay for 100 milliseconds
      delay(100);
      
      //spin the device twice to initiate the sampler
      SpinMe(2);
      
      Serial.println("Finished calibrating peristaltic pump sensor");
      i = 0;

}
bool netReg(void) {
  int ret;
  static char mccmnc[7] = "\0\0\0\0\0\0\0";
  static uint8_t network_searched = 0;

  if(mccmnc[0] == '\0'){//if uninitialised
    strncpy(mccmnc, MCCMNC, sizeof(mccmnc) -1 ); 
    mccmnc[sizeof(mccmnc) - 1] = '\0';
  }

  if (sendATcmd(F("AT+CFUN=1"), "OK", 10000, 2) == false) { // enable the modem
    return false;
  }

  if(ALLOW_GSM){
    sendATcmd(F("AT+CNMP=2"), "OK",
              500, 1); //enable GSM
  }else{
    sendATcmd(F("AT+CNMP=38"), "OK",
              500, 1); //disable GSM (its 2024!)
  }
            
  if(ALLOW_NBIOT){
     sendATcmd(F("AT+CMNB=3"), "OK",
               500, 1); //enable NB-IoT
  }else{
     sendATcmd(F("AT+CMNB=1"), "OK",
               500, 1); //disable NB-IoT (simbase does not offer NB-IoT)
  }

  sendATcmd(F("AT+CGDCONT=1,\"IP\",\"" APN "\""), "OK",
            2000); // set IPv4 and apn

  ret = isRegistered();
  if(ret){return ret;}

  ret = netSelect(mccmnc);
  if(ret){
      ret = getMccmnc(mccmnc, sizeof(mccmnc));
      if(ret){
        Serial.print(F("Updating network preference to: "));
        Serial.println(mccmnc);
      }
    return true; 
  }

  ret = netSelect(MCCMNC);
  if(ret){
      ret = getMccmnc(mccmnc, sizeof(mccmnc));
      if(ret){
        Serial.print(F("Updating network preference to: "));
        Serial.println(mccmnc);
      }
    return true; 
  }

  ret = netSelect(NULL);
  if(ret){
      ret = getMccmnc(mccmnc, sizeof(mccmnc));
      if(ret){
        Serial.print(F("Updating network preference to: "));
        Serial.println(mccmnc);
      }
    return true; 
  }
  
  if(!network_searched){
    Serial.println(F("Searching networks, this could take > 30 minutes."));
    netSearch();
    network_searched = true;//not we never do this again regardless of if the search was successfull
  }

  //attempt to connect to networks in order of signal strength
  for(uint8_t i = 0; i < operators_len; i++){
    Operator* o = &operators[i];
    if(o->rssi == 99){continue;}
    ret = netSelect(o->mccmnc);
    if(ret){
        ret = getMccmnc(mccmnc, sizeof(mccmnc));
        if(ret){
            Serial.print(F("Updating network preference to: "));
            Serial.println(mccmnc);
        }
        return true; 
    }
  }

  Serial.println(F("Unable to connect to network"));
  return false;
}
int netSelect(const char* mccmnc){
    int ret = 0;
    const uint32_t cops_timeout_ms = 150000;//120s specified in the AT command manual
    const uint32_t creg_timeout_ms = 150000;//this one is more how long we are willing to wait;

    if(mccmnc == NULL){
    	  Serial.println(F("Automatic network registration attempt"));
        ret = sendATcmd(F("AT+COPS=0"), "OK", cops_timeout_ms, 1);
        if(ret == 0){return false;}
    }else{
      	Serial.print(F("Attempting to register to: "));
	      Serial.println(mccmnc);
        char atcmd[64];
        snprintf(atcmd, sizeof(atcmd), "AT+COPS=4,2,%s", mccmnc);
        ret = sendATcmd(atcmd, "OK", cops_timeout_ms, 1);
        if(ret == 0){return false;}
    }
    
    ret = isRegistered(creg_timeout_ms);

    return ret;
}


void CBCread() {
  // get GNSS data
  if (sendATcmd(F("AT+CBC"), "OK", 1000)) {

    storeCBCresponse();
  }
}
int isRegistered(const uint32_t timeout_ms){
  int ret = 0;
  const uint32_t retry_delay_ms = 591;//empirically measured
  const uint32_t attempts = timeout_ms/retry_delay_ms;

  for(uint32_t i = 0; i < attempts; i++){
    ret = sendATcmd(F("AT+CREG?"), "OK", 1000, 1);
    if(ret){
      int registered = strstr(response, "+CREG: 0,5")
		       || (ACCEPT_NON_ROAMING && strstr(response, "+CREG: 0,1"));
      if(registered){
	  Serial.println(F("CREG: registered"));
	  return true;
      }
    }
    Serial.print(F("Waiting for registration, "));
    Serial.print(i);Serial.print(F("/"));Serial.println(attempts);
    xDelay(50);
  }
  Serial.println(F("CREG: not registered"));
  return false; 
}
void storeCBCresponse() {

  bool end = 0;
  uint8_t x = 0;
  uint8_t j = 0;

  memset(CBC, '\0', sizeof(CBC));

  // loop through reponce to extract data
  for (uint8_t i = 0; i < CHARBUFF; i++) {

    // string splitting cases
    switch (response[i]) {
      case ':':
        x = 9;
        j = 0;
        i += 2;
        break;

      case ',':
        x++;
        j = 0;
        break;

      case '\0':
        end = 1;
        break;
      case '\r':
        x++;
        j = 0;
        break;
    }
    // write to char arrays
    if (response[i] != ',') {
      switch (x) {
        case 11:
          CBC[j] = response[i];
          break;
      }
      // increment char array counter
      j++;
    }
    // break loop when end flag is high
    if (end) {
      i = CHARBUFF;
    }
  }
}

void CSQread() {
  uint8_t csq;
  getRSSI(&csq);
  memset(CSQ, '\0', sizeof(CSQ));
  sprintf(CSQ, "%u",  (unsigned)csq);
}

bool pullNetTime() {
  int ret = 0;

  if (sendATcmd(F("AT+CLTS=1"), "OK", 1000) == false) { // Tells the sim module to get the current time from cellular networrk and update the internal clock
    return false;
  }
  if (sendATcmd(F("AT+CCLK?"), "OK", 1000) == false) { // Aks the module "what time do you think it is? "
    return false;
  }
  char *p = strstr(response, "+CCLK: \"");
  if (p == NULL) {
    return false;
  }

  int16_t tz4;
  ret = sscanf(p, "%s \"%d/%d/%d,%d:%d:%d%d", response, &g_datetime.yr,
         &g_datetime.mt, &g_datetime.dy, &g_datetime.hr, &g_datetime.mn,
         &g_datetime.sc, &tz4);
  if(ret == 8){//number of matches)
    /* fix timezone offset to UTC */
    uint32_t y2ksec;
    datetime2y2k(&g_datetime, tz4, &y2ksec);
    y2k2datetime(&g_datetime, y2ksec);
    return true;
  }
  return false;
}

void _dbgPrintDateTime(const DATETIME& datetime) {
  Serial.print(datetime.yr);
  Serial.print("-");
  Serial.print(datetime.mt);
  Serial.print("-");
  Serial.print(datetime.dy);
  Serial.print(" ");
  Serial.print(datetime.hr);
  Serial.print(":");
  Serial.print(datetime.mn);
  Serial.print(":");
  Serial.print(datetime.sc);
  Serial.println();
}

void updateRTC(const DATETIME *dt) {
  if (dt == NULL) {
    MCP7940.adjust();
  } else {
    DateTime datetime = DateTime(dt->yr, dt->mt, dt->dy, dt->hr, dt->mn, dt->sc);
    MCP7940.adjust(datetime);
  }
  Serial.print(F("RTC Date/Time update to:"));
  DateTime now = MCP7940.now();
  char RTCBuffer[30];
  memset(RTCBuffer, '\0', sizeof(RTCBuffer));
  sprintf(RTCBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(),
          now.day(), now.hour(), now.minute(), now.second());
  Serial.println(RTCBuffer);
}


int netSearch(void){
    int ret = false;

    ret = sendATcmd(F("AT+COPS=?"), "OK", 1800000, 1);//I've observed this take as long as 20 minutes before so lets make the timeout 30 minutes
    if(!ret){return false;}

    storeOperators();
    _dbgPrintOperators();

    for(uint8_t i = 0; i < operators_len; i++){
        Operator* o = &operators[i];
        ret = netSelect(o->mccmnc);
        if(!ret){continue;}
        
        char mccmnc_current[7] = "\0\0\0\0\0\0\0";
        getMccmnc(mccmnc_current, sizeof(mccmnc_current));
        if(strcmp(o->mccmnc,mccmnc_current)){
          Serial.println(F("ERR: operators do not match"));
          continue;
        }

        getRSSI(&o->rssi);

    }

    qsort(operators, operators_len, sizeof(Operator), compare_rssi);  

    return true;
}
int getMccmnc(char* mccmnc, size_t mccmnc_size){
    int ret = 0;
    sendATcmd(F("AT+CPSI?"), "OK", 1000, 1);
    char mcc[4] = "\0\0\0\0";
    char mnc[4] = "\0\0\0\0";
    char* start = strchr(response, '+');
    if(start == NULL){return false;}

    ret = sscanf(start, "+CPSI: %*[^,],%*[^,],%3[^-]-%3[^,],", mcc, mnc);
    if(ret == 2){
      snprintf(mccmnc, mccmnc_size, "%s%s", mcc,mnc);
      return true;
    }
    return false;
}
// Wait for the "+HTTPACTION: 0,<code>,<len>" URC that the SIM7000 sends
// asynchronously ~1-15 seconds after AT+HTTPACTION=0.
// Returns true if <code> is 200 (success), false on timeout or error code.
bool waitForHttpAction(uint32_t timeout_ms) {
  unsigned long t0 = millis();
  char urc[96] = {0};
  uint8_t idx = 0;

  while (millis() - t0 < timeout_ms) {
    if (simCom.available()) {
      char c = simCom.read();
      if (idx < sizeof(urc) - 1) {
        urc[idx++] = c;
        urc[idx] = '\0';
      }

      // Look for a COMPLETE URC: "+HTTPACTION: 0,<code>,<len>\r\n"
      char *p = strstr(urc, "+HTTPACTION:");
      if (p != NULL) {
        int method = 0, code = 0, len = 0;
        int matched = sscanf(p, "+HTTPACTION: %d,%d,%d", &method, &code, &len);

        // Require all three numbers AND a trailing newline in the buffer
        // after the URC, which proves the line has finished arriving.
        bool line_complete = (strchr(p, '\n') != NULL);

        if (matched == 3 && line_complete) {
          Serial.print(F("URC +HTTPACTION code="));
          Serial.println(code);
          return (code == 200);
        }
      }

      // Bail early on a hard ERROR
      if (strstr(urc, "\r\nERROR\r\n") != NULL) {
        Serial.println(F("URC saw ERROR"));
        return false;
      }

      // Prevent buffer overflow: keep the last ~56 bytes
      if (idx >= sizeof(urc) - 1) {
        memmove(urc, urc + 40, sizeof(urc) - 40);
        idx = sizeof(urc) - 40;
        urc[idx] = '\0';
      }
    }
  }

  Serial.println(F("waitForHttpAction: timeout"));
  return false;
}


void setup(){
  wdt_disable();
  Serial.begin(BAUDRATE);
  lost_seconds = 0;
  initHardware();
  tryPowerOnSimCom();
  serialBegin();
  netReg();
  openbearer();
  Serial.println("GetMinsMaxs Started: ");
  GetMinsMaxs();
  Serial.println("GetMinsMaxs Ended: ");
  xDelay(2);
  Serial.println("NoOfRevolutions Started: ");
  revolutions = NoOfRevolutions();
  Serial.println("NoOfRevolutions Ended: ");
  Serial.println("Loop() Started: ");
  closeBearer();
  netUnreg();
  serialEnd();
  tryPowerOffSimCom();
}
void netUnreg() { sendATcmd(F("AT+CFUN=0"), "OK", 1000); }

bool logToWeb() {
  bool ret = false;
  static uint16_t upload_fails = 0;
  static uint16_t power_on_fails = 0;
  static uint16_t net_reg_fails = 0;

  Serial.print(F("upload fails: "));
  Serial.print(upload_fails);
  Serial.print(F(" power_on_fails: "));
  Serial.print(power_on_fails);
  Serial.print(F(" net_reg_fails: "));
  Serial.println(net_reg_fails);

  if (net_reg_fails > NET_REG_FAILS_LIMIT_BEGIN && 
      (net_reg_fails % NET_REG_FAILS_LIMIT_RATE)) {
    Serial.println(F("netReg failure high. Limiting logging"));
    net_reg_fails++;
    return false;
  }

  if (net_reg_fails > NET_REG_FAILS_LIMIT_BEGIN &&
     !(net_reg_fails % (3 * NET_REG_FAILS_LIMIT_RATE))) {
    Serial.println(F("netReg failure high. Reseting"));
    hardResetSimCom();
  }

  Serial.println(F("Initialising SIM 7000"));
  simCom.begin(BAUDRATE);

  uint8_t reset = false;
  for (uint8_t i = 0; i < TRY_POWER_ON_MAX_RESETS; i++) {
    if (reset == true) {
      reset = false; 
      power_on_fails++;
      if (power_on_fails < POWER_ON_FAILS_RESET_LIMIT_BEGIN || 
           !(power_on_fails % POWER_ON_FAILS_RESET_LIMIT_RATE)) {
          Serial.println(F("cannot communicate with simCom. resetting"));
          hardResetSimCom();
      }
    }
    
    ret = tryPowerOnSimCom(POWER_ON_TRY_TIMES);
    if(ret == false){reset = true; continue;}

    ret = serialBegin();
    if(ret == false){reset = true; continue;}
    
    break;
  }
  if(ret == false){goto cleanup;}

  ret = netReg();
  if (ret == false) {
    net_reg_fails++;
    goto net_reg_cleanup;
  } else {
    net_reg_fails = 0;
  }

  openbearer();
  CBCread();
  CSQread();

  if (pullNetTime()) {
    Serial.print("Updating to UTC time: ");
    _dbgPrintDateTime(g_datetime);
    updateRTC(&g_datetime);
  }

  for (uint8_t i = 0; i < HTTP_UPLOAD_TRY_TIMES; i++) {
    ret = sendAlert("0");
    if (ret == false) {
      upload_fails++;
      if (upload_fails > HTTP_FAILS_RESET_THRESHOLD) {
        upload_fails = 0;
        Serial.println(F("HTTP failure rate high. resetting"));
        hardResetSimCom();
        goto net_reg_cleanup;
      }
      continue;
    }
    break;
  }

  closeBearer();

net_reg_cleanup:
  netUnreg();

cleanup:
  serialEnd();
  tryPowerOffSimCom();
  Serial.print(F("LogToWeb() ret: "));
  Serial.println(ret);
  return ret;
}


void loop() {
  // ---------- If target already reached on a previous boot ----------
  if (currentML >= totalTargetVolume) {
    Serial.println(F("TARGET ALREADY REACHED. Sleeping until battery change."));
    sendAlertStandalone("1000");
    while (true) deepSleepSecs(8);
  }

  // ---------- Check the web every xdelay seconds ----------
  unsigned long checkStart = millis();
  bool web_ok = retrieveFromWeb();   // powers modem on, reads all vars (incl. xdelay), powers off

  if (!web_ok) {
    Serial.println(F("retrieveFromWeb failed — sleeping xdelay and retrying"));
    uint16_t sleep_secs = xdelay;
    if (sleep_secs < 30) sleep_secs = 30;
    deepSleepSecs(sleep_secs);           // fall back to whatever xdelay currently holds
    return;
  }

  // xdelay now holds the interval from the server. Convert to ms once.
  unsigned long interval_ms = (unsigned long)xdelay * 1000UL;

  // ---------- If server says stop, just idle ----------
  if (!state) {
    Serial.println(F("System is idle (state=0)! "));
    stopAllHardware();

    unsigned long elapsed_ms = millis() - checkStart;
    if (elapsed_ms < interval_ms) {
      unsigned long remaining_sec = (interval_ms - elapsed_ms) / 1000;
      if (remaining_sec > 0) deepSleepSecs(remaining_sec);
    }
    return;
  }

  // ---------- state == 1: run one full pump cycle ----------
  Serial.print(F("CYCLE STARTED: "));
  Serial.println(i + 1);

  bool cycle_ok = true;
  SpinMeRev(2 * revolutions);
  xDelay(5);

  if (!Rinse()) cycle_ok = false;
  else {
    xDelay(5000);
    SpinMeRev(2 * revolutions);
    xDelay(5000);
    if (!Rinse()) cycle_ok = false;
    else {
      xDelay(5000);
      SpinMeRev(2 * revolutions);
      xDelay(5000);
      if (!Rinse()) cycle_ok = false;
    }
  }

  if (cycle_ok) {
    xDelay(5000);
    int revs = (int) lround(ml_per_sample / ml_per_rev);
    SpinMe(revs);
    currentML += ml_per_sample;
    xDelay(5000);
    SpinMeRev(2 * revolutions);
    i++;
    logToWeb();
  } else {
    Serial.println(F("Rinse Failed"));
    sendAlertStandalone("200"); // Rinse Failed

    Serial.println(F("RINSE FAILED. Sleeping until battery changed. "));
    while (true) {
      deepSleepSecs(8);
    }
  }

  // ---------- Did we just reach the target? ----------
  if (currentML >= totalTargetVolume) {
    Serial.println(F("TARGET REACHED. Sleeping until battery change."));
    sendAlertStandalone("1000");// Target Reached
    while (true) deepSleepSecs(8);
  }

  // ---------- Timing ----------
  // If the cycle took >= xdelay, loop immediately.
  // Otherwise, sleep the remaining time in the interval.
  unsigned long elapsed_ms = millis() - checkStart;
  if (elapsed_ms < interval_ms) {
    unsigned long remaining_sec = (interval_ms - elapsed_ms) / 1000;
    if (remaining_sec > 0) {
      Serial.print(F("REMAINING SECS: "));
      Serial.println(remaining_sec);
      deepSleepSecs(remaining_sec);
    }
  }
  // else: fall through — loop() re-enters and checks the web again at once
}