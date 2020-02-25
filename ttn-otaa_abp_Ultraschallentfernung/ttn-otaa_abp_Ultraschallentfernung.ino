/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 *
 *
 *
 *Recommendation
 *THIS is definetly not for any PRODUCTIVE DEVICE
 *Keys are not stored encrypted.. Any idiot kann read out the eeprom to get the Data.
 *
 *HINTS:
 * Do not forget to define the radio type correctly in config.h.
 * found and adapted the OTAA from here:
 * https://github.com/Edzelf/LoRa/blob/master/ESP_lora_tracker/ESP_LoRa_tracker.ino
 
 //useful with this board
 Remarks: Cable from GPIO0 to NSS pin of RFM needed
 Cut Jumper-Line between GPIO16 and nss
 Solder jumper between GPIO16 and RST


 !!!!! IMPORTANT INFORMATION For Uploading Sketch!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 1.please UNPLUG USB first
 2.remove Lora Hub Board
 3. Upload
 4. Reconnect Lora Board
 5. Reconnect Power
 otherwise upload will fail due to connection between RST and GPIO16!
 *******************************************************************************/
//#define messungonly // for debugging of ultrasonic device - no Lora
//#define SingleChannelMode 1
//define debug_messages
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoraMessage.h> //https://github.com/thesolarnomad/lora-serialization/blob/master/src/LoraMessage.h
#include <EEPROM.h>

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 180; //Time between transmissions in SECONDS


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0xFILLMEIN_LSB, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xFILLMEIN_LSB };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format. In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xFILLMEIN_MSB };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
//static osjob_t sendjob;

int trigPin = 4;    // Trigger
int echoPin = 5;    // Echo
long duration, cm;
int tx_status = 0;


u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ; 

#define DATAVALID 0xACF2AFC2                     // Pattern for data valid in EEPROM/RTC memory
                                                 // Change if you want OTA to renew keys.

struct savdata_t                                 // Structure of data to be saved over reset
{
  uint32_t dataValid ;                           // DATAVALID if valid data (joined)
  uint8_t  devaddr[4] ;                          // Device address after join
  uint8_t  nwkKey[16] ;                          // Network session key after join
  uint8_t  artKey[16] ;                          // Aplication session key after join
  uint32_t seqnoUp ;                             // Sequence number                      
} ;

//***************************************************************************************************
// Global data.                                                                                     *
//***************************************************************************************************
bool      OTAA = true ;                          // Assume connection through OTAA
uint32_t  data[2] ;                              // Data to TTN: N and E integer (multiplied by 1E6)
osjob_t   initjob ;                              // Lmic handle for init job
osjob_t   sendjob ;                              // Lmic handle for send job
savdata_t savdata ;                              // Data to be saved over reset
bool      sleepreq = false ;                     // Request to go to sleep

//*********************************************************************/
//Lora Pinning for WEMOS D1 Addon Board
 /*
  GPIO12 (D6) <----> MISO
  GPIO13 (D7) <----> MOSI
  GPIO14 (D5) <----> CLK
  GPIO15 (D8) <----> DIO0/D2 OR DIO1/D3 OR DIO2/D4
  GPIO16 (D0) <----> SEL Chip Select (depending on bottom solder PAD position)
   */
  #define LORA_PIN_SPI_MOSI 13
  #define LORA_PIN_SPI_MISO 12
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  0 //16 //Dummer Fehler den Pin 16 für CHip Select zu nehmen https://github.com/matthijskooijman/arduino-lmic/pull/34
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO  15


const lmic_pinmap lmic_pins = {
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LORA_PIN_SPI_DIO, LORA_PIN_SPI_DIO, LMIC_UNUSED_PIN}, 
    //workaround to use 1 pin for all 3 radio dio pins
};



//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t i ;                                                        // Loop control
#ifdef debug_messages
  Serial.print ( header ) ;                                           // Show header
  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        Serial.printf ( "\n" ) ;                                      // Yes, print it
      }
      Serial.printf ( "%04X: ", i ) ;                                 // Print index
    }
    Serial.printf ( "%02X ", *p++ ) ;                                 // Print one data byte
  }
  Serial.println() ;
#endif
}


//***************************************************************************************************
//                                    S A V E T O R T C                                             *
//***************************************************************************************************
// Save data in RTC memory.  Every 100th call the data will also be saved in EEPROM memory.         *
// The EEPROM is also updates if OTAA was used.                                                     *
// The space in RTC memory is limited to 512 bytes.                                                 *
//***************************************************************************************************
void saveToRTC()
{
  uint16_t        eaddr ;                                  // Address in EEPROM
  uint8_t*        p ;                                      // Points into savdata

  Serial.printf ( "Save data to RTC memory\n" ) ;
  memcpy ( savdata.devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( savdata.nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( savdata.artKey,  LMIC.artKey, 16 ) ;
  savdata.seqnoUp = LMIC.seqnoUp ;
  savdata.dataValid = DATAVALID ;
  memdmp ( "devaddr:", savdata.devaddr, 4 ) ;
  memdmp ( "nwkKey:",  savdata.nwkKey, 16 ) ;
  memdmp ( "artKey:",  savdata.artKey, 16 ) ;
  Serial.printf ( "SeqnoUp is %d\n", savdata.seqnoUp ) ;
  Serial.printf ( "SeqnoDown is %d\n", LMIC.seqnoDn ) ;
  ESP.rtcUserMemoryWrite ( 0, (uint32_t*) &savdata, sizeof(savdata) ) ;
  if ( ( ( LMIC.seqnoUp % 100 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    Serial.println ( "Saving to EEPROM" ) ;
    p = (uint8_t*)&savdata ;                               // set target pointer
    for ( eaddr = 0 ; eaddr < sizeof(savdata) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
    }
    EEPROM.commit() ;                                      // Commit data to EEPROM
  }
}


//***************************************************************************************************
//                                    I N I T F U N C                                               *
//***************************************************************************************************
static void initfunc (osjob_t* j)
{
    // reset MAC state
    Serial.println ( "Reset MAC" ) ;
    LMIC_reset();
    LMIC_setLinkCheckMode ( 0 ) ;
    LMIC_setDrTxpow ( DR_SF7, 14 ) ;
    if ( OTAA )
    {
      // start joining
      Serial.println ( "Start joining" ) ;
      LMIC_startJoining();
    }
    else
    {
      memdmp ( "Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
      Serial.printf ( "Seqnr set to %d\n", savdata.seqnoUp ) ;
      LMIC_setSession ( 0x1, DEVADDR, NWKSKEY, APPSKEY ) ;
      LMIC.seqnoUp = savdata.seqnoUp ;                      // Correction counter
      do_send ( &sendjob ) ;
    }
    Serial.println ( "Initfunc finished" ) ;
    // init done - onEvent() callback will be invoked...
}
//***************************************************************************************************
//***************************************************************************************************

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
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

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            tx_status = 2;
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
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            tx_status = 3;
            sleepreq = true ; 
            savdata.seqnoUp++; 
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
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: No JoinAccept"));
            break;    
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}


//***************************************************************************************************
//                                R E T R I E V E K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys en seqnr from non-volitile memory.                                      *
//***************************************************************************************************
void retrieveKeys()
{
  uint16_t eaddr ;                                          // Address in EEPROM
  uint8_t* p ;                                              // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  ESP.rtcUserMemoryRead ( 0, (uint32_t*)&savdata,           // Retriev saved data from RTC memory
                          sizeof(savdata) ) ;

  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    Serial.println ( "Keys retrieved from RTC memory\n" ) ; // Show retrieve result 
  }
  else
  {
    // No data vailable in RTC memory.  Use EEPROM data.
    p = (uint8_t*)&savdata ;
    for ( eaddr = 0 ; eaddr < sizeof(savdata) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
    }
    savdata.seqnoUp += 100 ;                                // Counter may be not up-to-date
  }
  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf ( "Valid data in NVS\n" ) ;               // Yes, show
    memdmp ( "devaddr is:",
             savdata.devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             savdata.nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             savdata.artKey, 16 ) ;
    Serial.printf ( "seqnr is %d\n", savdata.seqnoUp ) ;
    memcpy ( (uint8_t*)&DEVADDR,
             savdata.devaddr, sizeof(DEVADDR) ) ;          // LoraWAN DEVADDR, end node device address
    memcpy ( NWKSKEY,
             savdata.nwkKey,  sizeof(NWKSKEY) ) ;          // LoRaWAN NwkSKey, network session key.
    memcpy ( APPSKEY,
             savdata.artKey,  sizeof(APPSKEY) ) ;          // LoRaWAN AppSKey, application session key.
    
    OTAA = false ;                                         // Do not use OTAA
  }
  else
  {
    Serial.printf ( "No saved data, using OTAA\n" ) ;
  }
}
//***************************************************************************************************
//***************************************************************************************************
//***************************************************************************************************
//                                do send                                                           *
// send the actual data every TX_Interval
//***************************************************************************************************
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        LoraMessage message;
        message.addUint16(int(cm));
        
        //message.addHumidity(
        LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
        
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//***************************************************************************************************
//***************************************************************************************************
//                                     Init Lora Stack, sets ADR Mode, 
//***************************************************************************************************

void setup_lora(void){
  // LMIC init
    os_init();
    
    LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
    //Debugging Purpose for Single Channel
    #ifdef SingleChannelMode
    Serial.println ( "\n\n CAUTION Single Channel MODE \n\n" ) ;
      #define CHANNEL  1
      for (uint8_t i = 0; i < 9; i++) {
        if (i != CHANNEL) {
          LMIC_disableChannel(i);
        }
      }
    #endif
    
    //Adaptive Data Rate Mode https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate.html 
    //LMIC_setAdrMode(1); //Adaptiert Datenrate nach 64 Paketen
    LMIC_setLinkCheckMode(0);

    
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    
    // Set data rate and transmit power for uplink moved depending if OTAA or ABP
    //LMIC_setDrTxpow(DR_SF12,14);//Langsamster Modus: elendslange AirTime ~820ms aber sichere Übertragung ASozial für alle anderen User da das den Kanal voll macht!!!
    //LMIC_setDrTxpow(DR_SF8,14); //Kurz schnell unzuverlässig 
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

    if ( OTAA )
    {
      // start joining
      Serial.println ( "Start joining" ) ;
      //VERSUCH! LMIC_setDrTxpow(DR_SF9,14); //with SF9 initial Join is faster... yes I know for this Reset Cycle it will stay on SF9 and consume more airtime
      do_send (&sendjob) ;
    }
    else
    {
      //Serial.printf ( "starte mit SF8: gespeichertes SAVED_seqnoUp: %d\n", SAVED_seqnoUp ) ;
      LMIC_setDrTxpow(DR_SF8,14); //otherwise stupid ABP will start with SF12 and consume a lot of airtime!!!
      memdmp ( "No OTAA needed - Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
       
      LMIC_setSession ( 0x13, DEVADDR, NWKSKEY, APPSKEY ) ;
      Serial.printf ( "Seqnr recovered to %d\n", savdata.seqnoUp ) ;
      LMIC.seqnoUp = savdata.seqnoUp ;
      do_send (&sendjob) ;
    }
    
    
    //Eingebaut in /src/lmic.c geht nicht
    //LMIC_dn2dr = EU868_DR_SF9;//https://github.com/mcci-catena/arduino-lmic/issues/455
    //LMIC_selectSubBand(0); //https://github.com/mcci-catena/arduino-lorawan/issues/74
    // Start job (sending automatically starts OTAA too)
}
//***************************************************************************************************
//***************************************************************************************************
//                   Performs Ultrasonic detection on a HR SR4 Module
//***************************************************************************************************
void do_ultrasonic_measurement(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343 for speed of sound
  Serial.print(cm); Serial.println(" cm Abstand erfasst");  
}

//***************************************************************************************************
//***************************************************************************************************
void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting of INIT all Systems"));
    EEPROM.begin ( 512 ) ;  
    
    pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
    
    do_ultrasonic_measurement(); 
    
    #ifdef VCC_ENABLE
        // For Pinoccio Scout boards
        pinMode(VCC_ENABLE, OUTPUT);
        digitalWrite(VCC_ENABLE, HIGH);
        delay(1000);
    #endif
    
    #ifdef messungonly
        Serial.println(F("Nur Ultraschall kein LORA, kein TTN, nur für debugging"));
    #else
        retrieveKeys() ;    
        setup_lora();
    
        // Start job (sending automatically starts OTAA too)
        do_send(&sendjob);
    #endif
}
//***************************************************************************************************
//***************************************************************************************************

//***************************************************************************************************
//***************************************************************************************************

void loop() {    
      
    #ifdef messungonly
      do_ultrasonic_measurement();
      tx_status = 3;
      delay(500);
    #else
      os_runloop_once();
      uint32_t sleeptime ;                                        // Time to sleep to next sample
      uint32_t tnow ;                                             // Current runtime in microseconds
      
      os_runloop_once() ;
      if ( sleepreq )                                             // Time to go to sleep?
      {
        tnow = millis() * 1000 ;                                  // Current runtime in microseconds
        saveToRTC() ;                                             // Save to RTC memory
        sleeptime = TX_INTERVAL * 1000000 ;                       // Sleeptime in microseconds
        if ( sleeptime > tnow )                                   // Prevent negative sleeptime
        {
          sleeptime = sleeptime - tnow ;                          // Correction for duration of program
        }
        Serial.printf ( "Going to sleep for %ld seconds....",
                        sleeptime / 1000000 ) ;
        ESP.deepSleep ( sleeptime, WAKE_RF_DEFAULT ) ;            // Sleep for about tx_Interval minutes
        // Will never arrive here...
      }
    #endif
    
    
}
