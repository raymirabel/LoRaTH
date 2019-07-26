/**************************************************************************
 @FILE:         LoRaTH.ino
 @AUTHOR:       Raimundo Alfonso
 @COMPANY:      Ray Ingeniería Electronica, S.L.
 @DESCRIPTION:  Ejemplo de uso para el nodo LoRaTH y emoncms (https://emoncms.org)
                Example of use for the LoRaSDL node and emoncms (https://emoncms.org)
  
 @LICENCE DETAILS:
  Este sketch está basada en software libre. Tu puedes redistribuir
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
  17-07-2019 - v1.00 : Primera versión
  
**************************************************************************/

#define  FIRMWARE_VERSION "1.00"
#define  HARDWARE_VERSION "190506"

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RHReliableDatagram.h>     // https://www.airspayce.com/mikem/arduino/RadioHead/index.html
#include <RH_RF95.h>                // https://www.airspayce.com/mikem/arduino/RadioHead/index.html
#include <LowPower.h>               // https://github.com/rocketscream/Low-Power
#include <OneWire.h>                // https://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>      // https://www.arduinolibraries.info/libraries/dallas-temperature
#include <Adafruit_BMP280.h>        // https://github.com/adafruit/Adafruit_BMP280_Library
#include <SHT2x.h>                  // https://github.com/raymirabel/SHT2x

// Defines what sensors are available in the hardware...
#define SHT21
//#define DS18B20
//#define SUNRISE
//#define BMP280    // Note: When the pressure sensor is present, the pressure value replaces the dew temperature value (see payload structure)


// Structure of default configuration parameters. This parameters are stored in internal eeprom...
typedef struct {
  int     sendTime        = 10;     // [1...9999] Send time delay (minutes)
  byte    rfPower         = 14;     // [5...23]   RF power (5:min...23:max)
  byte    rfRetries       = 3;      // [0...20]   Data send retries (0 = no retries)
  byte    rfNode          = 0;      // [0...250]  Node id
  byte    rfPan           = 100;    // [0...250]  PAN id (Only nodes with this same number are visible)
  boolean dipsw           = true;   // [on|off]   rfNode dipswitch mode:
                                    //            - true:  rfNode = dipswitch + rfPan (the rfNode parameter is ignored)
                                    //            - false: rfNode = rfNode + rfPan 
  boolean led             = true;   // [on|off]   led test mode:
                                    //            - true:  led test mode turn on when the node reads sensors and transmits payload.
                                    //            - false: led test is always off 
} stConfig;
stConfig config;

// Identify the node
//#define DEVICE_ID      5      // 5 = LoRaSDL
#define DEVICE_ID      6      // 6 = LoRaTH
//#define DEVICE_ID      7      // 7 = LoRaX1
//#define DEVICE_ID      8      // 8 = LoRaHALL

// Hardware definitions...
#define SERVER_ADDRESS  250
#define RX              0
#define TX              1
#define RFM95_INT       2
#define RDY_SUN         3
#define RFM95_CS        4
#define RFM95_GPIO1     5
#define RX_SUN          6
#define TX_SUN          7
#define EN_SENS         8
#define LED             9
#define EN_SUN          10
#define DIPSW0          A0
#define DIPSW1          A1
#define DIPSW2          A2
#define DIPSW3          A3
#define PIN_DS18B20     A4
#define VBAT            A6
#define LDR             A7
#define DIR_EEPROM_CFG  10

// Payload structure...   
typedef struct {
  byte        pan_id;
  byte        device_id = DEVICE_ID;
  int         status;         // bit 0:  SHT21 temperature & humidity sensor OK or present
                              // bit 1:  DS18B20 temperature sensor OK or present       
                              // bit 2:  sunrise CO2 sensor OK or present  
                              // bit 3:  LDR sensor OK or present     
                              // bit 4:  BMP280 sensor OK or present
                              // ...
                              // bit 15: device timeout
  int         rssi;           // x1   - dB
  int         temperature;    // x10  - ºC
  int         humidity;       // x1   - %
#ifndef BMP280  
  int         dew_point;      // x10  - ºC
#else
  int         pressure;       // x10  - hPa
#endif  
  int         co2;            // x1   - ppm
  int         light;          // x1   - %
  int         battery;        // x100 - V
} Payload;
Payload theData;

// RFM95 transceiver configuration and instances...
#define RF95_FREQ 868.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, 0);

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

// BMP280...
#ifdef BMP280
  Adafruit_BMP280 bmp; // I2C
#endif

// Global variables...
boolean modoCMD = false;  // flag line command entry

/**************************************************************************
 * SETUP
 *************************************************************************/  
void setup(){
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

  // Check if the eeprom is empty...
  if(EEPROM.read(0) == 123){
    // Read the configuration parameters...
    EEPROM.get(DIR_EEPROM_CFG, config);
  }else{
    // If it is empty, it saves the configuration parameters by default...
    EEPROM.write(0,123);
    EEPROM.put(DIR_EEPROM_CFG, config);
  }

  // Init serial port...
  while (!Serial);
  Serial.begin(9600);

#ifdef SHT21
  // Init I2C bus...
  Wire.begin();
  // Desactivate internal pullups for I2C.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0); 
#endif  

#ifdef DS18B20  
  ds18b20.begin();
  ds18b20.setWaitForConversion(true);
  ds18b20.setResolution(12);
#endif

#ifdef SUNRISE
  // Init software serial port...
  sun.begin(9600);
#endif  

#ifdef BMP280
  sens_on();
  delay(100);
  if (!bmp.begin(0x77, 0x58)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  // Default settings...
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // Desactivate internal pullups for I2C.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);                    
  sens_off();                  
#endif


  // Init payload struct...
  theData.pan_id   = config.rfPan;
#ifndef BMP280  
  theData.dew_point = 0;
#else
  theData.pressure = 0;
#endif  
  theData.co2 = 0;
  theData.temperature = 0;
  theData.humidity = 0;
  theData.light = 0;
  theData.status = 0; 

  // init RFM95 module...
  radioInit();
  rf95.sleep();
  delay(10);  


  // Check the RX pin to see if you have to enter command line mode...
  if(digitalRead(RX)){
    modoCMD = true;
    led_commandLine();
    commandLine();
    modoCMD = false;
  }
  led_init();

  // Sleep 8 seconds before transmitting the first frame after power-up...
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}
/**************************************************************************
 * LOOP
 *************************************************************************/
void loop(){
  // Turn on test led. This led shows us how long the node is awake...
  led_test_on();

  // Sensors read...
  lee_sensores();
      
  // Send payload to server...
  send_to_server();

  // Turn of test led...
  led_test_off();
 
  // Sleep x minutes...
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  duerme(config.sendTime);
}

/**************************************************************************
 * FUNCTIONS
 *************************************************************************/ 

void lee_sensores(void){ 
  // Read battery level...
  lee_bateria();

  // Read light level...
  lee_ldr();  

  // Read temperature and humidy sensor...
#ifdef SHT21  
  sens_on();
  leeSHT21();
  sens_off();
#else
  theData.status &= ~0x01;
#endif  

  // Read temperature sensor...
#ifdef DS18B20 
  sens_on();
  leeDS18B20();
  sens_off();
#else
  theData.status &= ~0x02;
#endif  

  // Read CO2 sensor...
#ifdef SUNRISE
  leeCO2();
#else
  theData.status &= ~0x04;
#endif

  // Read pressure BMP280 sensor...
#ifdef BMP280
  leeBMP280();
#else
  theData.status &= ~0x10;
#endif


}

void sens_on(void){
  digitalWrite(EN_SENS, HIGH);
  delay(10);
}

void sens_off(void){
  digitalWrite(EN_SENS, LOW); 
}


#ifdef BMP280
void leeBMP280(void){
  sens_on();
  delay(10);
  bmp.begin(0x77, 0x58);
  theData.pressure = bmp.readPressure() / 10;
  theData.status |= 0x10; 
  // Desactivate internal pullups for I2C.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);   
  sens_off();
}
#endif

#ifdef SHT21
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
#ifndef BMP280     
    theData.dew_point = calcDewpoint(h,t);
#endif    
    theData.status |= 0x01; 
  }else{
    theData.temperature = 0;
    theData.humidity = 0;  
#ifndef BMP280          
    theData.dew_point = 0;
#endif       
    theData.status &= ~0x01;
  }
}

int calcDewpoint(float humi, float temp) {
  float k;
  
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return (int)((243.12 * k / (17.62 - k)) * 10.0);
}
#endif

#ifdef DS18B20
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
  }
}
#endif

#ifdef SUNRISE
void leeCO2(void){
  char buffer_rx[15];
  char cnt;
  int co2;
  byte n;
  boolean co2_ok = false;

  // Activa el sensor...
  digitalWrite(EN_SUN, HIGH);
  delay(500);

  // Apaga led para ahorrar miestras se realiza el muestreo...
  led_test_off();
  
  // Espera a que se realize el muestreo... apaga el micro mientras esperas...
  if(modoCMD)
    delay(8000);
  else
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
  }else{
    theData.co2 = 0;    
    theData.status &= ~0x04;
  }  

  // Desactiva el sensor...
  digitalWrite(EN_SUN, LOW);
}
#endif

void lee_ldr(void){
  sens_on();
  delay(1);
  theData.light = 100 - (unsigned int)(((unsigned long)(analogRead(LDR)) * 100L) / 1023L);
  theData.status |= 0x08; 
  sens_off();
}

void lee_bateria(void){
  unsigned long valores = 0;
  byte n;

  delay(1);
  // Lee 5 muestras...
  for(n=0;n<5;n++){
     valores += (unsigned int)(((unsigned long)(analogRead(VBAT)) * 330L) / 1023L);
     delay(1);
  }
  theData.battery = valores / 5;
}

