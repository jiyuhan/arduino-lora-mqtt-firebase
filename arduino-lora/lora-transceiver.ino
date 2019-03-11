/*******************************************************************************


   This example sends LoRaWAN packets containing sensor readings at periodic
   intervals.  The sensor values are: temperature, barometric pressure, and
   relative humidity, obtained from a BME280 sensor chip, and luminosity obtained
   from a BH1750 LUX sensor.  Sensor data is coded in CayenneLPP format

   The LoRaWAN stack implementation is provided by the Arduino-LMIC library
   (things-nyc version) which is a port of the IBM LMIC Library.

   This example uses OTAA (Over-the-air activation), where a DevEUI and
   application key are configured and used in an over-the-air
   join procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

    *******************************************************************************/
//Libraries for LMIC (LoRaWAN stack)
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//library for RTCZero (real time clock for sleep (standby) mode)
#include <RTCZero.h>

//library for CayenneLPP (conversion to CayenneLPP data format)
#include <CayenneLPP.h>

//libraries for BME_280 sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//library for BH1750 Lux Sensor (also needs Wire.h but was included earlier)
#include <BH1750.h>

// Disable printing for production, enable for testing w. USB connection.
// Note: Due to unknown issues involving the interplay of sleep (standby) mode
// with USB functionality, printing will work only up until the first
// sleep period (which follows the first packet transmission).
// You can disable sleep mode by commenting out a clearly marked section of 
// code in the EV_TXCOMPLETE case of the on_Event() function and replacing it
// with a statement that schedules the next packet for transmission without
// going into sleep mode.
// Be sure to do this only for debugging purposes as the processor will consume
// MUCH more power in this mode.
//COMMENT OUT the #define to disable printing
#define PRINTING_ENABLED

//pin definitions for BME_280 (not needed for I2C)
//#define BME_SCK 13
//#define BME_MISO 12
//#define BME_MOSI 11
//#define BME_CS 10
#define VBATPIN A7
//Instantiate BME280 object for I2C
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

//You must supply the APPEUI, DEVEUI, and APPKEY for your TTN application.
//These values will be generated when you create your TTN application and
//add a device to it.

// The EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from the TTN, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// The key shown here is the semtech default key.  Replace it with the
// key for your TTN application
static const u1_t PROGMEM APPKEY[16] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

static bool send_battery = true;

//instantiate buffer for CayenneLPP
CayenneLPP lpp(51);

//Instantiate real time clock (RTC) object for sleep
RTCZero rtc;
// This is the interval between packet transmissions.
//It can be set between 1 and 59 minutes
const unsigned SLEEP_MINUTES = 1;
int alarmTime;   //time to wakeup from sleep (minutes)

// instantiate BH1750 object
BH1750 luxSensor;

// Pin mapping for LoRa radio.
// Note: To use the LoRa radio on the
// Feather M0, a connection is needed between
// pins DIO1 and D6.  This connection is provided
// by the MCCI Catena wing, but must be supplied
// if Feather/LoRa is used without the wing.

const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  //  .rst = LMIC_UNUSED_PIN,
  .dio = {3, 6, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
#ifdef PRINTING_ENABLED
  Serial.print(os_getTime());
  Serial.print(": ");
#endif
  switch (ev) {
    case EV_SCAN_TIMEOUT:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_SCAN_TIMEOUT"));
#endif
      break;
    case EV_BEACON_FOUND:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_BEACON_FOUND"));
#endif
      break;
    case EV_BEACON_MISSED:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_BEACON_MISSED"));
#endif
      break;
    case EV_BEACON_TRACKED:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_BEACON_TRACKED"));
#endif
      break;
    case EV_JOINING:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_JOINING"));
#endif
      break;
    case EV_JOINED:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_JOINED"));
#endif
      //LMIC_setSeqnoUp(140);
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      //Seems critical to turn off Link Check Mode
      LMIC_setLinkCheckMode(0);
      //Turn off data rate adaptation
      LMIC_setAdrMode(0);
      //Set spreading factor & TA power (only if ADR disabled)
      // LMIC_setDrTxpow(DR_SF7, 14);
      LMIC_setDrTxpow(DR_SF10, 14);
      //set downlink data rate for TTN (MUST BE THE SAME AS THE UPLINK RATE)
      LMIC.dn2Dr = DR_SF7;
      //Take initial sensor readings and transmit first frame
      do_send(&sendjob);
      break;
    case EV_RFU1:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_RFU1"));
#endif
      break;
    case EV_JOIN_FAILED:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_JOIN_FAILED"));
#endif
      break;
    case EV_REJOIN_FAILED:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_REJOIN_FAILED"));
#endif
      break;
    case EV_TXCOMPLETE:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_TXCOMPLETE"));
#endif
      if (LMIC.txrxFlags & TXRX_ACK)
#ifdef PRINTING_ENABLED
        Serial.println(F("Received ack"));
#endif
      //LMIC.dataLen > 0 means downlink data was received
      if (LMIC.dataLen) {
#ifdef PRINTING_ENABLED
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.print(F(" bytes of payload: "));
#endif
        //LMIC.dataBeg gives offset of downlink data
        //in LMIC.frame
        for (int i = 0; i < LMIC.dataLen; i++) {
          //HEX format does not print leading zeros
          //so have to do it explicitly
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
#ifdef PRINTING_ENABLED
            Serial.print("0");
#endif
          }
#ifdef PRINTING_ENABLED
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
          Serial.print(" ");
#endif
        }
#ifdef PRINTING_ENABLED
        Serial.print("Battery switch: ");
        Serial.print(LMIC.frame[LMIC.dataBeg + 2]);
        Serial.println();
#endif
        if(LMIC.frame[LMIC.dataBeg + 2] == 0x64) {
#ifdef PRINTING_ENABLED
          Serial.println("Battery info in display");
#endif
          send_battery = true;
        }

        if(LMIC.frame[LMIC.dataBeg + 2] == 0x00) {
#ifdef PRINTING_ENABLED
          Serial.println("Battery info not in display");
#endif
          send_battery = false;
        }
        
        Serial.println();
      }
      // Sleep for SLEEP_MINUTES minutes.
      // Unfortunately, sleep mode breaks the USB so no more
      // printing after first sleep.  If you need to keep printing
      // active for debugging purposes, you can disable sleep mode
      // by commenting out the section of code below and uncommenting
      // the other designated code.
  // TO DISABLE SLEEP MODE, COMMENT OUT THE SECTION OF CODE BEGINNING HERE

#ifdef PRINTING_ENABLED
      Serial.print("Sleeping for ");
      Serial.print(SLEEP_MINUTES);
      Serial.println(" minutes");
#endif

      alarmTime = rtc.getMinutes();
      alarmTime += SLEEP_MINUTES;  // time for next wakeup interrupt
      alarmTime = alarmTime % 60; // correct for minute roll-over
      rtc.setAlarmMinutes(alarmTime);  // wake up at alarm time--every SLEEP_MINUTES minutes
      rtc.enableAlarm(rtc.MATCH_MMSS);  // match minutes and seconds
      rtc.attachInterrupt(alarmMatch);  // attach call-back function to interrupt;
      rtc.standbyMode();  //sleep until next alarm match
#ifdef PRINTING_ENABLED
      Serial.println( "Waking up to send next packet");
#endif
      // Awake. Time to collect and send more data
      do_send(&sendjob);
  // END OF THE SECTION OF CODE TO BE COMMENTED OUT TO DISABLE SLEEP MODE

  // IF SLEEP MODE IS DISABLED, UNCOMMENT THE FOLLOWING STATEMENT
  // OTHERWISE IT MUST BE COMMENTED OUT
      // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(60*SLEEP_MINUTES), do_send);

      break;
    case EV_RESET:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_RESET"));
#endif
      break;
    case EV_RXCOMPLETE:
#ifdef PRINTING_ENABLED
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
#endif
      break;
    case EV_LINK_DEAD:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_LINK_DEAD"));
#endif
      break;
    case EV_LINK_ALIVE:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_LINK_ALIVE"));
#endif
      break;
    case EV_LOST_TSYNC:
#ifdef PRINTING_ENABLED
      Serial.println(F("EV_LOST_TSYNC"));
#endif
      break;
    default:
#ifdef PRINTING_ENABLED
      Serial.println(F("Unknown event"));
#endif
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
#ifdef PRINTING_ENABLED
    Serial.println(F("OP_TXRXPEND, not sending"));
#endif
  }
  else {
    //Prepare upstream data transmission at the next possible time.
    //Force BME280 sensor read
    bme.takeForcedMeasurement();  //Reads temp, pressure, humidity into BME280 buffers
    //reset buffer for LoRa packet payload
    lpp.reset();
    //populate LoRa packet payload from BME280 buffers; translate to CayenneLPP format

    float temp = bme.readTemperature();
#ifdef PRINTING_ENABLED
    Serial.println(temp);
#endif
    lpp.addTemperature(1, temp);

    float humid = bme.readHumidity();
#ifdef PRINTING_ENABLED
    Serial.println(humid);
#endif
    lpp.addRelativeHumidity(2, humid);

    float pressure = bme.readPressure() / 100.0F;
#ifdef PRINTING_ENABLED
    Serial.println(pressure);
#endif
    lpp.addBarometricPressure(3, pressure);

    //Read LUX sensor and add to LoRa packet payload
    uint16_t lux = luxSensor.readLightLevel();
#ifdef PRINTING_ENABLED
    Serial.println(lux);
#endif
    lpp.addLuminosity(4, lux);
    if(send_battery) {
        int sensorValue = analogRead(A4);
        // analog reading to voltage 3.3V
        float measuredvbat = sensorValue * 3.3 / 1024;
        if(measuredvbat - 0.4 < 0.0) measuredvbat = 0;
        else measuredvbat -= 0.4;
        float windspeed = (measuredvbat)*32.4*3.6/1.6;
        
        // float measuredvbat = analogRead(VBATPIN);
        // measuredvbat *= 2;
        // measuredvbat *= 3.3;
        // measuredvbat /= 1024;
#ifdef PRINTING_ENABLE
        Serial.println(windspeed);
#endif
        lpp.addAnalogInput(5, windspeed);
    }

    // run once
    // lpp.addDigitalOutput(6, 1);


    //Queue LoRa packet for immediate transmission
    //as permitted by duty cycle restriction
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
#ifdef PRINTING_ENABLED
    Serial.println(F("Packet queued"));
#endif
  }
}

void setup() {
#ifdef PRINTING_ENABLED
  Serial.begin(9600);
  Serial.println(F("Starting"));
#endif

  //Initialize RTC for timing sleep mode
  rtc.begin();

  //Initialize BME sensor
  bool status;
  // default settings
  //BME begin initializes Wire, so
  status = bme.begin();
  if (!status) {
#ifdef PRINTING_ENABLED
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
#endif
    while (1);
  }
  //Configure BME280 for Forced Mode (see Section 3.5 of BME280 datasheet)
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, //temperature
                  Adafruit_BME280::SAMPLING_X1, //pressure
                  Adafruit_BME280::SAMPLING_X1, //humidity
                  Adafruit_BME280::FILTER_OFF );
#ifdef PRINTING_ENABLED
  Serial.println("-- BME Sensor init successful --");;
  Serial.println();
#endif

  //initialize BH1750 lux sensor; no need to initialize I2C (Wire) since
  //it's done by bme.begin
  luxSensor.begin();
#ifdef PRINTING_ENABLED
  Serial.println("BH1750 init. successful");
  Serial.println();
#endif

  // Initialize LMIC LoRaWAN stack
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  //Enable/Disable adaptive data rate

  //Select SubBand 0. Our gateways only implement this sub-band
  LMIC_selectSubBand(0);

  // Initiate LoRaWAN join using OTAA.  First packet will be
  // sent after join is complete (EV_JOINED)
  LMIC_startJoining();
}

void alarmMatch() {  //callback for RTC alarm interrupt
  //This is not used for anything but is required by RTCZero syntax.
}

void loop() {
  os_runloop();
}
