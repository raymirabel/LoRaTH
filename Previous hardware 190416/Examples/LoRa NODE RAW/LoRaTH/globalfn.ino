// RFM95 radio init...
void radioInit(void){
  while (!manager.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
#ifdef DEBUG
  Serial.println("LoRa radio init OK!");
#endif

  if(config.dipsw){
    manager.setThisAddress(leeDIPSW());
  }else{
    manager.setThisAddress(config.rfNode);
  }
  manager.setRetries(config.rfRetries);
  manager.setTimeout(100);
#ifdef DEBUG  
  Serial.print("Set client address = ");
  Serial.println(leeDIPSW());
#endif

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  }
#ifdef DEBUG  
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
#endif  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(config.rfPower, false);
}

// Send payload data to server...
boolean send_to_server(void){
  boolean r;

  r = manager.sendtoWait((const void*)(&theData), sizeof(theData), SERVER_ADDRESS);
  rf95.sleep();
  delay(10);
  return(r);
}


// Sleep...
void duerme(int minutos){
  int m = (minutos * 60) / 8;
  int n;
  for(n=0;n<m;n++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}


// Read dipswitch
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
  
  // Turn off pull-up resistor to save energy...
  pinMode(DIPSW0,INPUT);
  pinMode(DIPSW1,INPUT);  
  pinMode(DIPSW2,INPUT);  
  pinMode(DIPSW3,INPUT);    

  return(a0 + a1*2 + a2*4 + a3*8);
}

void led_commandLine(void){
  for(byte n=0;n<3;n++){
    led_test_on();
    delay(50);
    led_test_off();
    delay(50);
  }
}

void led_init(void){
    led_test_on();
    delay(50);
    led_test_off();
}


void led_test_on(void){
  if(config.led) digitalWrite(LED, LOW);
}

void led_test_off(void){
  digitalWrite(LED, HIGH);
}

// Filter function...
int filterIIR (int data, byte f){
  static boolean ini = true;
  static long data_ant = 0;
  long data_now;
  long output;

  data_now = (long)data * 10L;

  // Captura la primera muestra...
  if(ini){
    data_ant = data_now;
    ini = false;
  }

  // Filtra...
  data_ant = ((data_now * (10L - f)) + (data_ant * (f))) / 10L;
  output = data_ant / 10L;

  return(output);
}

void i2DecimalPoint(int numero, byte decimales, char *cadena){
  byte len;
  int i;
  int x;
  char format[5];

  x = decimales - len + 1;
  sprintf(format,"%%0%dd", x);
   
  // Pasa el número a cadena...
  sprintf(cadena,format,numero);
  len = strlen(cadena);
  
  // Recorre toda la cadena hasta que encuentres el punto...
  for(i = len + 0; i>=0; i--){
    cadena[i+1] = cadena[i];
    if(i == len - decimales){
      cadena[i] = '.';
      break;
    }    
  }
}
