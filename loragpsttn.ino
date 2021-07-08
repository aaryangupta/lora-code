// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

//DEBUGGING
#define DEBUG
#define DEBU//G_XL

//Includes
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//LoRaWAN credentials
#ifdef CREDENTIALS
static const u1_t NWKSKEY[16] = NWKSKEY1;
static const u1_t APPSKEY[16] = APPSKEY1;
static const u4_t DEVADDR = DEVADDR1;
#else
static const u1_t NWKSKEY[16] = {0x3f, 0x3F, 0x0A,  0xF7, 0xEE,3, 0xAC, 0xC2, 0x9E, 0x38, 0x29, 0x25, 0x29, 0x88, 0x30 };
static const u1_t APPSKEY[16] = { 0x93,  0xC9,  0x15,  0xF8,  0x54, 0x3A, 0x8E, 0x7F, 0x93, 0x61, 0xD9, 0x30 };
static const u4_t DEVADDR = 0x89E;
#endif

uint8_t mydata[14]; //Extra bytes as buffer to play with
const unsigned message_size =11;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Sendjob
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 250;
dr_t LMIC_DR_sequence[] = {DR_SF7, DR_SF10, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF9, DR_SF7, DR_SF7, DR_SF7, DR_SF7};
int LMIC_DR_sequence_count = 12;
int LMIC_DR_sequence_index = 0;

// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins = {
.nss = 10,
.rxtx = LMIC_UNUSED_PIN,
.rst = 9,
.dio = {2, 6, 7},
};

// TinyGPS object
TinyGPS gps;

//SoftwareSerial pin mapping
SoftwareSerial ss(3, 2); // RX, TX  Arduino RX, TX --> GPS TXD, RXD

void onEvent (ev_t ev) {
Serial.print(os_getTime());
Serial.print(": ");
//Error catching
switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    //Disable link check validarion (Auto enabled during join, but not supported by TTN);
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (Includes waiting for RX windows"));
    if (LMIC.txrxFlags & TXRX_ACK) { // No { ???
      Serial.println(F("Received ack"));
    }
    if (LMIC.dataLen) {
      Serial.println(F("Recieved "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    //Recieved data in ping slot
    Serial.println(F("EV_RX_COMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    //If error != debug then display default value
    Serial.println(F("What the fuck is going on"));
    break;
    
}
}

// do_send call is scheduled in event handler
void do_send(osjob_t* j){
  // starting version == martijn's version

Serial.println("\ndo_send was called");

// Check if there is not a current TX/RX job running
if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
} else {
    // Prepare upstream data transmission at the next possible time.
    // Debug for sending

    #ifdef DEBUG
    Serial.println("  expected   CA DA F? 83 5E 9? 0 ?? ?? " );  
    Serial.println("    dummy   7F FF FF 7F FF FF 0 0 0 " );    
    #endif  
    Serial.print(" mydata[] = [");
    Serial.print( mydata[0], HEX );
    Serial.print(" ");
    Serial.print( mydata[1], HEX );
    Serial.print(" ");
    Serial.print( mydata[2], HEX );
    Serial.print(" ");
    Serial.print( mydata[3], HEX );
    Serial.print(" ");
    Serial.print( mydata[4], HEX );
    Serial.print(" ");
    Serial.print( mydata[5], HEX );
    Serial.print(" ");
    Serial.print( mydata[6], HEX );
    if (message_size>7) Serial.print(" ");
    if (message_size>7) Serial.print( mydata[7], HEX );
    if (message_size>8) Serial.print(" ");
    if (message_size>8) Serial.print( mydata[8], HEX );
    if (message_size>9) Serial.print(" / ");
    if (message_size>9) Serial.print( mydata[9], HEX );
    if (message_size>10) Serial.print(" ");
    if (message_size>10) Serial.print( mydata[10], HEX );
    Serial.print("]    ");
    
    Serial.print("DR [ ");
    Serial.print( LMIC_DR_sequence_index );
    Serial.print(" ] = ");
    Serial.print( LMIC_DR_sequence[LMIC_DR_sequence_index] );
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF7) Serial.print(" DR_SF7 "); 
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF8) Serial.print(" DR_SF8 "); 
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF9) Serial.print(" DR_SF9 "); 
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF10) Serial.print(" DR_SF10 "); 
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF11) Serial.print(" DR_SF11 "); 
    if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF12) Serial.print(" DR_SF12 "); 

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library
    // for the ttn mapper always use SF7. For other use cases, SF12 can be used, however that will require 60 mins quiet time,
    // according to TTN rules.
    LMIC_setDrTxpow(LMIC_DR_sequence[LMIC_DR_sequence_index],14); // Set data rate

    LMIC_DR_sequence_index = LMIC_DR_sequence_index + 1;
    if (LMIC_DR_sequence_index >= LMIC_DR_sequence_count) {
      LMIC_DR_sequence_index=0;
    }

    // Send that shit
    LMIC_setTxData2(1, mydata, message_size, 0);

    Serial.println(" - packet queued");
    
}
// Next TX is scheduled after TX_COMPLETE event.
}

void lmic_init() {
  os_init();
  //Reset MAC state
  LMIC_reset();

  //EU config
  #if defined(CFG_eu868)
  // Set up the channels for TTN
  // Corresponds to most default EU gateways
  // Good for debugging, doesn't overload frequencies
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  #elif defined(CFG_us915)
  //USA frequency config
  LMIC_selectSubBand(1);
  #endif
  LMIC_setLinkCheckMode(0); //Enable/disable link check validation

  // TTN uses SF9 for its RX2 window
  LMIC.dn2Dr = DR_SF9;
  //Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7,14);
  
}

void setup() {
Serial.begin(115200);
Serial.println(F("Starting..."));

//Load the send buffer with dummy location. 0,0 is recognized as dummy by TTN and will therefor not be displayed.
put_gpsvalues_into_sendbuffer( 0,0,0,0);

Serial.println();
Serial.println();
Serial.println("Starting GPS LoRa transmission");
Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
Serial.println();

//GPS Serial
ss.begin(9600); //Software serial with GPS module.

#ifdef VCC_ENABLE
  //For sleep mode
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
 #endif

 lmic_init();

 //Start job delayed so system can look for GPS first
 os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(5), do_send);

// Set static session parameters.
LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
}

void loop() {

// process the serial feed from GPS module
Serial.println(" ");
//Serial.println("Read GPS... ");
char c;
unsigned long start = millis();
do 
{   
  while (ss.available())
  {
    char c = ss.read();
    #ifdef DEBUG_XL
    Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    #endif
    
    if (gps.encode(c)) // Did a new valid sentence come in?
        process_gps_values();
  }
} while (millis() - start < 3000); // if too high a value then system wil delay scheduled jobs and the send sequence will take too long
 
os_runloop_once();  // system picks up scheduled jobs
}

void process_gps_values()
{ 
  // retrieve some usefull values from GPS library
  float flat, flon, alt;
  unsigned long age; 
  int hdopNumber;  
  
  gps.f_get_position(&flat, &flon, &age);  // lat -90.0 .. 90.0 as a 4 byte float, lon -180 .. 180 as a 4 byte float, age in 1/1000 seconds as a 4 byte unsigned long
  alt = gps.f_altitude();    // signed float altitude in meters
  hdopNumber = gps.hdop();   // int 100ths of a meter

  // check if possibly invalid
  bool GPS_values_are_valid = true;
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)    GPS_values_are_valid = false;
  if (flon == TinyGPS::GPS_INVALID_F_ANGLE)    GPS_values_are_valid = false;
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) GPS_values_are_valid = false;
  if (age == TinyGPS::GPS_INVALID_AGE)         GPS_values_are_valid = false;
  
  if (alt == TinyGPS::GPS_INVALID_F_ALTITUDE)  GPS_values_are_valid = false;   // if alt, hdop remain giving errors, possibly the GPS character read misses every start few characters of every feed. Solution: make the code lighter so it returns quicker to character read. Or process a bit of buffer while doing other actions, see TinyGPS example.

  // if valid, put into buffer
  if (GPS_values_are_valid) put_gpsvalues_into_sendbuffer( flat, flon, alt, hdopNumber);
// after init, sendbuffer holds 0,0 lovation; after first fix it will retain the last valid location
  
  Serial.print(".");
  //show me something
  #ifdef DEBUG
  // keep some values out as seems to take performance and/or make for code to miss GPS sentences
  unsigned long chars = 0;
  //unsigned short sentences = 0, failed = 0;
  //uint32_t sat; 
  //gps.stats(&chars, &sentences, &failed);
  //sat = gps.satellites();
  
  Serial.println();
  Serial.print("Data: ");
  if (GPS_values_are_valid) Serial.print("(valid) ");
  if (!GPS_values_are_valid) Serial.print("(** INVALID");
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE)    {Serial.print(" lat="); Serial.print(flat);}
  if (flon == TinyGPS::GPS_INVALID_F_ANGLE)    {Serial.print(" lon="); Serial.print(flon);}
  if (hdopNumber == TinyGPS::GPS_INVALID_HDOP) {Serial.print(" hdop="); Serial.print(hdopNumber);}
  if (age == TinyGPS::GPS_INVALID_AGE)         {Serial.print(" age="); Serial.print(age);}
  if (alt == TinyGPS::GPS_INVALID_F_ALTITUDE)  {Serial.print(" alt="); Serial.print(alt);}
  if (!GPS_values_are_valid) Serial.print(" **) ");
  Serial.print("  LAT, LON=");
  Serial.print( flat, 6);   
  Serial.print(", ");
  Serial.print(flon, 6); // 52.632656, 4.738389
  Serial.print(" hdop=");
  Serial.print( hdopNumber);
  Serial.print(" alt=");
  Serial.print( alt );
  Serial.print(" AGE=");
  Serial.print(age);
  //Serial.print(" SAT=");
  //Serial.print( sat);
  
  //  Serial.print(" CHARS=");
  //  Serial.print(chars);
  //Serial.print(" SENT=");
  //Serial.print(sentences);
  //Serial.print(" ERR=");
  //Serial.print(failed);
  Serial.println("");
  #endif
  
  #ifdef DEBUG_XL
  if (chars == 0)
Serial.println("** No characters from GPS: check wiring **");
  else if (age > 5000)
Serial.println("Warning: possible stale GPS data (age over 5 seconds)");
  else
Serial.println("GPS Data is fresh (age less than 5 seconds)");

  Serial.print("For TTN message LatitudeBinary, LongitudeBinary, altitudeGps, accuracy: ");
//    Serial.print( LatitudeBinary, HEX);
//    Serial.print(", ");
//    Serial.print( LongitudeBinary, HEX );
//    Serial.print(", ");
//    Serial.print( altitudeGps, HEX );
//    Serial.print(", ");
//    Serial.println( accuracy, HEX );
  Serial.println("expected   CA DA F. 83 5E 9. 0 .. .. " );     
  Serial.println("    dummy   7F FF FF 7F FF FF 0 0 0 " );    
  Serial.print(  "mydata[] = ");
  Serial.print( mydata[0], HEX );
  Serial.print(" ");
  Serial.print( mydata[1], HEX );
  Serial.print(" ");
  Serial.print( mydata[2], HEX );
  Serial.print(" ");
  Serial.print( mydata[3], HEX );
  Serial.print(" ");
  Serial.print( mydata[4], HEX );
  Serial.print(" ");
  Serial.print( mydata[5], HEX );
  Serial.print(" ");
  Serial.print( mydata[6], HEX );
  if (message_size>6) Serial.print(" ");
  if (message_size>6) Serial.print( mydata[7], HEX );
  if (message_size>7) Serial.print(" ");
  if (message_size>7) Serial.print( mydata[8], HEX );
  if (message_size>8) Serial.print(" / ");
  if (message_size>8) Serial.print( mydata[9], HEX );
  if (message_size>9) Serial.print(" ");
  if (message_size>9) Serial.print( mydata[10], HEX );
  Serial.println("]");
  #endif    
}



void put_gpsvalues_into_sendbuffer(float flat, float flon, float alt, int hdopNumber)
{  
  
  uint32_t LatitudeBinary = ((flat + 90) / 180) * 16777215;
  uint32_t LongitudeBinary = ((flon + 180) / 360) * 16777215;
  uint16_t altitudeGps = alt;         // altitudeGps in meters, alt from tinyGPS is float in meters
  if (alt<0) altitudeGps=0;   // unsigned int wil not allow negative values and warps them to huge number, needs to be zero'ed
  // uint8_t accuracy = hdopNumber*10;   // needs to be /10 instead of *10 as per example JP
  uint8_t accuracy = hdopNumber/10;   // from TinyGPS horizontal dilution of precision in 100ths, TinyGPSplus seems the same in 100ths as per MNEMA string
  
  mydata[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  mydata[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  mydata[2] = LatitudeBinary & 0xFF;

  mydata[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  mydata[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  mydata[5] = LongitudeBinary & 0xFF;

  // altitudeGps in meters into unsigned int
  mydata[6] = ( altitudeGps >> 8 ) & 0xFF;
  mydata[7] = altitudeGps & 0xFF;

  // hdop in tenths of meter
  mydata[8] = accuracy & 0xFF;
  
  mydata[9] = 0;  // fill up next bytes in buffer, just for play. As-if null terminated string.
  mydata[10] = 0xFF;  // dummy filler byte
}
