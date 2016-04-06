//GPS & SD
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

// Accel / Gyro / Mag
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

//GPS
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false
#define LOG_FIXONLY true  
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//SD
File logfile;
#define chipSelect 10 //sd uses pin 10


byte dataLoopInterval = 10;
signed long currentTime;
signed long previousTime;
signed long minDiff;

// keeping track of the number of ignition pulses between logs
signed int ignitionPulses = 0;
void pinThreeInterupt() {
  ignitionPulses++;
}

void initLogger(){
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
    Serial.println("Card init. failed!");
  }
  char filename[15];
  strcpy(filename, "JMLLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); Serial.println(filename);
  }
  Serial.print("Writing to "); Serial.println(filename);
}

void initGPS(){
  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // How often we refresh 1HZ, 5HZ or 10HZ
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);  // How often we get a new fix 5HZ or 1HZ
  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
  useInterrupt(true);
  Serial.println(F("GPSReady!"));
}


void initLSM()
{
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void readGPS(){
 // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    
    // Sentence parsed! 
    if (LOG_FIXONLY && !GPS.fix) {
        Serial.print("No Fix");
        return;
    }
    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    logfile.write((uint8_t *)stringptr, stringsize);   //write the string to the SD file
  }
}


void readLSM(){
  
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  // print out accelleration data
  // silly lib is off by an order of mag on the data... it will report .98 m/s^2 instead of 9.8 m/s^2
  logfile.print(accel.acceleration.x * 10 ); 
  logfile.print(",");
  logfile.print(accel.acceleration.y * 10 ); 
  logfile.print(",");
  logfile.print(accel.acceleration.z * 10 );
  logfile.print(",");
 
  // print out magnetometer data
  logfile.print(mag.magnetic.x);
  logfile.print(",");
  logfile.print(mag.magnetic.y);
  logfile.print(",");
  logfile.print(mag.magnetic.z);
  logfile.print(",");
  
  // print out gyroscopic data
  logfile.print(gyro.gyro.x); 
  logfile.print(",");
  logfile.print(gyro.gyro.y); 
  logfile.print(",");
  logfile.print(gyro.gyro.z);
  logfile.print(",");
  
  // temp
  logfile.println(temp.temperature);
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__


int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("start setup"));
  Serial.println(freeRam());

  Serial.println(F("init logger"));
  initLogger();
  Serial.println(freeRam());

  Serial.println(F("init gps"));
  initGPS();
  Serial.println(freeRam());

  Serial.println(F("init lsm"));
  initLSM();
  Serial.println(freeRam());

  //pin 2 is interupt 0 pin 3 is interupt 1
  attachInterrupt(1, pinThreeInterupt, RISING);  //this is the ignition interupt

  Serial.println(F("end setup"));
}

void loop()  {
  currentTime = millis();
  if(currentTime >= (previousTime + minDiff)){
    readGPS();
    logfile.print(currentTime); logfile.print(",");
    readLSM();
    // it will auto flush to card after 512 bytes so you can risk no save on the last 512 and have it flush at random times
    logfile.flush(); 
    previousTime = currentTime;
  }
}
