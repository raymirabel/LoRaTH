 /**************************************************************************
 @FILE:           LoRaTH_ttn.ino
 @AUTHOR:         Raimundo Alfonso
 @COMPANY:        Ray Ingeniería Electronica, S.L.
 @DESCRIPTION:    
  Sketch ejemplo de uso protocolo LoRaWAN para sensor LoRaTH
  LoRaWAN example sketch for LoRaTH sensor.        
  
  This example sends a valid LoRaWAN packet, using frequency and encryption 
  settings matching those of the The Things Network (https://www.thethingsnetwork.org)
                
  This uses ABP (Activation-by-personalisation), where a DevAddr and
  Session keys are preconfigured (unlike OTAA, where a DevEUI and
  application key is configured, while the DevAddr and session keys are
  assigned/generated in the over-the-air-activation procedure).
 
  Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
  g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
  violated by this sketch when left running for longer and low tx interval)!
 
  To use this sketch, first register your application and device with
  the things network, to set or generate a DevAddr, NwkSKey and
  AppSKey. Each device should have their own unique values for these
  fields.
 
  Do not forget to define the radio type correctly in config.h (lmic library)
   
 @LICENCE DETAILS:
  Este sketch está basado en software libre. Tu puedes redistribuir
  y/o modificar esta sketch bajo los términos de licencia GNU.

  Esta programa se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los términos de licencia GNU para más detalles:
                                                                       
  http://www.gnu.org/licenses/gpl-3.0.txt

  This sketch is based on free software. You can redistribute
  and/or modify this library under the terms of the GNU license.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY, even without the implied warranty of
  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU license terms for more details:   
  
  http://www.gnu.org/licenses/gpl-3.0.txt

 @VERSIONS:
  - HARDWARE COMPATIB. : 190113 - v1.01
    30-04-2019 - v1.00 : initial version
  
**************************************************************************/


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SHT2x.h>

//*****************************************************************************************************************************************
//*****************************************************************************************************************************************

// LoRaWAN NwkSKey, network session key (formato MSB)
// This is the default Semtech key, which is used by the early prototype TTN network.
static const PROGMEM u1_t NWKSKEY[16] = {  };

// LoRaWAN AppSKey, application session key (formato MSB)
// This is the default Semtech key, which is used by the early prototype TTN network.
static const u1_t PROGMEM APPSKEY[16] = {  };

// LoRaWAN end-device address (DevAddr) (formato MSB)
static const u4_t DEVADDR = 0x000000; // <-- Change this address for every node!

// Schedule TX in minutes.
const unsigned TX_INTERVAL = 1;

// Sensors present in hardware...
#define SHT21
//#define DS18B20
//#define SUNRISE

//*****************************************************************************************************************************************
//*****************************************************************************************************************************************

// Device identification number by Ray...
#define DEVICE_ID      6      // 5 = LoRaSDL
                              // 6 = LoRaTH

// Payload structure by Ray...
typedef struct {
  byte        device_id = DEVICE_ID;
  int         status;         // bit 0:  SHT21 temperature & humidity sensor OK or present
                              // bit 1:  DS18B20 temperature sensor OK or present       
                              // bit 2:  sunrise CO2 sensor OK or present       
                              // ...
                              // bit 15: device timeout
  int         temperature;    // x10  - ºC
  int         humidity;       // x1   - %
  int         dew_point;      // x10  - ºC
  int         co2;            // x1   - ppm
  int         dipswitch;      // x1   - binary code
  int         battery;        // x100 - V
} Payload;
Payload theData;

//*****************************************************************************************************************************************
//*****************************************************************************************************************************************


// Hardware ATMEGA328P pin mapping...
#define RX          0
#define TX          1
#define RFM95_GPIO0 2
#define RDY_SUN     3
#define RFM95_CS    4
#define RFM95_GPIO1 5
#define RX_SUN      6
#define TX_SUN      7
#define EN_SENS     8
#define LED         9
#define EN_SUN      10
#define DIPSW0      A0
#define DIPSW1      A1
#define DIPSW2      A2
#define DIPSW3      A3
#define PIN_DS18B20 A4
#define VBAT        A6

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
boolean end_tx = false;


// RFM95 pin mapping
const lmic_pinmap lmic_pins = {
  .nss = RFM95_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {RFM95_GPIO0, RFM95_GPIO1, LMIC_UNUSED_PIN},
};

// SHT21...
#ifdef SHT21
SHT2xClass SHT2x;
#endif

// DS18B20...
#ifdef DS18B20
  OneWire oneWire(PIN_DS18B20);
  DallasTemperature ds18b20(&oneWire);
#endif

// SUNRISE...
#ifdef SUNRISE
  SoftwareSerial sun(TX_SUN, RX_SUN);
#endif
    
void setup() {
  pinMode(EN_SENS, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(DIPSW0,INPUT);
  pinMode(DIPSW1,INPUT);  
  pinMode(DIPSW2,INPUT);  
  pinMode(DIPSW3,INPUT);  
  pinMode(RDY_SUN, INPUT);
  pinMode(EN_SUN,OUTPUT);
 
  sens_off();
  led_test_off();

  // Serial port init...
  while (!Serial);
  Serial.begin(9600);
  Serial.println(F("Starting"));

#ifdef SHT21
  // Temperature and humidity sensor I2C bus init...
  Wire.begin();
  // desactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);  
#endif  

#ifdef DS18B20  
  // Temperature sensor init...
  ds18b20.begin();
  ds18b20.setWaitForConversion(true);
  ds18b20.setResolution(12);
#endif


#ifdef SUNRISE
  // CO2 sunrise serial port init...
  sun.begin(9600);
#endif  

  // Payload init...
  theData.dew_point = 0;
  theData.co2 = 0;
  theData.temperature = 0;
  theData.humidity = 0;
  theData.status = 0; 
  theData.dipswitch = 0;
 
  // lmic library init...
  lmic_init();
  
  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();

  if(end_tx){
    end_tx = false;
    led_test_off();
    delay(100);
    // A echar una cabezada x minutos...
    duerme(TX_INTERVAL);   
   
    // lmic library init...
    lmic_init();
    do_send(&sendjob);
  }
}

// Send payload...
void do_send(osjob_t* j){
  char* ptStr;
  
  // Enciende led de test para ver cuando tiempo está despierto el sensor...
  led_test_on();

  // Lee sensores...
  lee_sensores();
 
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      ptStr = reinterpret_cast<char*>(&theData);
      LMIC_setTxData2(1, ptStr, sizeof(theData), 0);
      Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

// Read all sensors...
void lee_sensores(void){ 
  // Lee dipswitch...
  theData.dipswitch = leeDIPSW();
  
  // Lee nivel de bateria...
  lee_bateria();

  // Lee sensor temperatura y humedad...
#ifdef SHT21  
  sens_on();
  leeSHT21();
  sens_off();
#else
  theData.status &= ~0x01;
#endif  

  // Lee sensor temperatura...
#ifdef DS18B20 
  sens_on();
  leeDS18B20();
  sens_off();
#else
  theData.status &= ~0x02;
#endif  

  // Lee sensor de CO2...
#ifdef SUNRISE
  leeCO2();
#else
  theData.status &= ~0x04;
#endif
}

// Sleep x minutes...
void duerme(int minutos){
  int m = (minutos * 60) / 8;
  int n;
  for(n=0;n<m;n++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

// Dipswitch read...
byte leeDIPSW(void){
  byte a0,a1,a2,a3;
  pinMode(DIPSW0,INPUT_PULLUP);
  pinMode(DIPSW1,INPUT_PULLUP);  
  pinMode(DIPSW2,INPUT_PULLUP);  
  pinMode(DIPSW3,INPUT_PULLUP);    
  
  // Lee dipswitch...
  a0 = !digitalRead(DIPSW0);  
  a1 = !digitalRead(DIPSW1);
  a2 = !digitalRead(DIPSW2);
  a3 = !digitalRead(DIPSW3);
 
  // Quita pull-up para ahorrar bateria...
  pinMode(DIPSW0,INPUT);
  pinMode(DIPSW1,INPUT);  
  pinMode(DIPSW2,INPUT);  
  pinMode(DIPSW3,INPUT);    

  // Calcula dirección...
  return(a0 + a1*2 + a2*4 + a3*8);
}

void led_test_on(void){
  digitalWrite(LED, LOW);
}

void led_test_off(void){
  digitalWrite(LED, HIGH);
}

void sens_on(void){
  digitalWrite(EN_SENS, HIGH);
  delay(10);
}

void sens_off(void){
  digitalWrite(EN_SENS, LOW);
}


#ifdef SHT21
// Temperature and humidity sensor read...
void leeSHT21(void){
  unsigned long timeOutSensor = 0; 
  float t;
  float h;
  boolean temperature_ok = false;
  boolean humidity_ok = false;
 
  SHT2x.PrepareTemperatureNoHold();
  timeOutSensor = millis();  
  do{ 
    if(SHT2x.GetTemperatureNoHold(&t)){
      temperature_ok = true;     
    }
  }while(temperature_ok == false && (millis() - timeOutSensor) < 100L);
  
  SHT2x.PrepareHumidityNoHold();
  timeOutSensor = millis();  
  do{ 
    if(SHT2x.GetHumidityNoHold(&h)){
      humidity_ok = true;   
    }
  }while(humidity_ok == false && (millis() - timeOutSensor) < 100L);

  if(temperature_ok && humidity_ok){
    theData.temperature = (int)(t * 10);
    theData.humidity = (int)h;  
    theData.dew_point = calcDewpoint(h,t);
    theData.status |= 0x01; 
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" ºC");
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");    
  }else{
    theData.temperature = 0;
    theData.humidity = 0;      
    theData.dew_point = 0;
    theData.status &= ~0x01;
  }


}

// Dewpoint function
float calcDewpoint(float humi, float temp) {
  float k;
  
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return (243.12 * k / (17.62 - k));
}
#endif

#ifdef DS18B20
// Temperature sensor read...
void leeDS18B20(void){
  float temperature;

  ds18b20.requestTemperatures(); 
  temperature = ds18b20.getTempCByIndex(0);
  if((int)temperature >= 85){
    theData.temperature = 0;
    theData.status &= ~0x02;
  }else{
    theData.temperature = (int)(temperature * 10);
    theData.status |= 0x02;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" ºC");
  }
}
#endif

#ifdef SUNRISE
// CO2 sensor read...
void leeCO2(void){
  char buffer_rx[15];
  char cnt;
  int co2;
  byte n;
  boolean co2_ok = false;

  // Activa el sensor...
  digitalWrite(EN_SUN, HIGH);
  delay(500);

  // Espera a que se realize el muestreo... apaga el micro mientras esperas...
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  

  // Comando lectura de CO2... lee un par de veces...
  for(n=0;n<2;n++){  
    sun.write(0xFE);
    sun.write(0x04);
    sun.write((byte)0x00);
    sun.write(0x03);
    sun.write((byte)0x00);
    sun.write(0x01);  
    sun.write(0xD5);
    sun.write(0xC5); 
    delay(50);
    
    buffer_rx[0] = 0;
    cnt = 0;  
    co2_ok = false;
    while(sun.available()){
      buffer_rx[cnt++] = sun.read();
      if(cnt >= 7){
        co2 = word(buffer_rx[3], buffer_rx[4]);
        co2_ok = true;
      }
    }
    delay(10);
  }

 if(co2_ok){
    theData.co2 = co2;    
    theData.status |= 0x04;
    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.println(" ppm");    
  }else{
    theData.co2 = 0;    
    theData.status &= ~0x04;
  }  

  // Desactiva el sensor...
  digitalWrite(EN_SUN, LOW);
}
#endif

// Battery voltage read...
void lee_bateria(void){
  unsigned long valores = 0;
  byte n;

  delay(1);
  // Lee 5 muestras...
  for(n=0;n<5;n++){
     valores += (unsigned int)(((unsigned long)(analogRead(VBAT)) * 330L) / 1023L);
     delay(1);
  }
  
  valores /= 5;
  theData.battery = valores;
  Serial.print("Battery: ");
  Serial.print((float)valores / 100.0);
  Serial.println(" V");
}



// Disables all channels, except for the one defined above, and sets the
// data rate (SF). This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
//
// Not LoRaWAN compliant; FOR TESTING ONLY!
//
void forceTxSingleChannelDr() {
  int channel = 0;
  int dr = DR_SF7;

  for (int i = 0; i < 9; i++) { // For EU; for US use i<71
    if (i != channel) {
      LMIC_disableChannel(i);
    }
  }
  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 14);
}

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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }      
            end_tx = true; 
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void lmic_init(void){
    // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF7,14);
  
  // Define the single channel and data rate (SF) to use
  forceTxSingleChannelDr();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1/ 100);
}

