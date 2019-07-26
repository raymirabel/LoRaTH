#include <SerialCMD.h>

//Check these values in the SerialCMD.h file of the SerialCMD library:
/*
#define CMD_MAX_COMMANDS        20    // Máximo numero de comandos permitidos
#define CMD_MAX_COMMANDS_LENGHT 10    // Longitud máxima de caracteres por cada comando
#define CMD_MAX_COMMANDS_ARGS   3     // Número máximo de argumentos por comando
#define CMD_MAX_INPUT_LENGHT    50    // Tamaño del buffer de entrada de comandos y argumentos
*/


// Command line class instance...
SerialCMD cmd;  

void commandLine(void){
  cmd.begin(Serial, F("LoRaTH>"), welcome);

  // Añade comandos...
  cmd.addCommand(command_pow,   F("pow"),       F("[5...23]   RF power (5:min...23:max)"));
  cmd.addCommand(command_stime, F("stime"),     F("[1...9999] Send time delay (minutes)"));
  cmd.addCommand(command_retri, F("retri"),     F("[0...20]   Data send retries"));  
  cmd.addCommand(command_node,  F("node"),      F("[0...250]  Node id"));
  cmd.addCommand(command_dip,   F("dip"),       F("[on|off]   Dipswitch node id function"));   
  cmd.addCommand(command_led,   F("led"),       F("[on|off]   Led test mode"));    
  cmd.addCommand(command_pan,   F("pan"),       F("[0...250]  PAN id"));
  cmd.addCommand(command_read,  F("read"),      F("Read sensors"));
  cmd.addCommand(command_send,  F("send"),      F("Read and send sensors data to server"));
  cmd.addCommand(command_param, F("all"),       F("Read all parameters"));
  cmd.addCommand(command_help,  F("?"),         F("Command list help"));  

  // Stay in a loop while you have the TTL programmer connected...
  while(1){
    if(!digitalRead(RX)){
      delay(100);
      if(!digitalRead(RX)) break;
    }
    cmd.task();
  }
}

// Welcome...
void welcome(void){
  cmd.println(F("Ray Ingenieria Electronica, S.L."));
  cmd.print  (F("LoRaTH - Firm.: v"));
  cmd.print  (F(FIRMWARE_VERSION));
  cmd.print  (F(" Hard.: "));
  cmd.println(F(HARDWARE_VERSION));  
  cmd.println(F("Press '?' for help")); 
}

/**************************************************************************
 * COMMAND FUNCTIONS
 *************************************************************************/ 
// Comando pow...
int command_pow(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfPower;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 5, 23, &temp)){
      config.rfPower = (byte)temp;
      rf95.setTxPower(config.rfPower, false);
      EEPROM.put(DIR_EEPROM_CFG, config);
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("pow = %ld\r\n"),(long)config.rfPower);
  return(ok);
}

// Comando stime...
int command_stime(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.sendTime;  
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 1, 9999, &temp)){
      config.sendTime = (byte)temp;
      EEPROM.put(DIR_EEPROM_CFG, config);
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("stime = %ld\r\n"),(long)config.sendTime);
  return(ok);
}

// Comando retri...
int command_retri(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfRetries;  
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 20, &temp)){
      config.rfRetries = (byte)temp;
      manager.setRetries(config.rfRetries);
      EEPROM.put(DIR_EEPROM_CFG, config);    
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("retri = %ld\r\n"),(long)config.rfRetries);
  return(ok);
}

// Comando node...
int command_node(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfNode;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 250, &temp)){
      config.rfNode = (byte)temp;
      if(config.dipsw){
        manager.setThisAddress(leeDIPSW());
      }else{
        manager.setThisAddress(config.rfNode);
      }
      EEPROM.put(DIR_EEPROM_CFG, config);        
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("node = %ld\r\n"),(long)config.rfNode);
  return(ok);  
}


// Comando dip...
int command_dip(int argc, char** argv){
  bool ok = false;
  boolean temp;

  temp = config.dipsw;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testBoolean(argv[1], &temp)){
      config.dipsw = temp;
      if(config.dipsw){
        manager.setThisAddress(leeDIPSW());
      }else{
        manager.setThisAddress(config.rfNode);
      }      
      EEPROM.put(DIR_EEPROM_CFG, config);  
      ok = true;
    }
  }  
  if(ok){
    if(config.dipsw)
      cmd.print(F("dipsw = on\r\n"));
    else
      cmd.print(F("dipsw = off\r\n"));
  }
  return(ok); 
}

// Comando led...
int command_led(int argc, char** argv){
  bool ok = false;
  boolean temp;

  temp = config.led;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testBoolean(argv[1], &temp)){
      config.led = temp;
      EEPROM.put(DIR_EEPROM_CFG, config);  
      ok = true;
    }
  }  
  if(ok){
    if(config.led)
      cmd.print(F("led = on\r\n"));
    else
      cmd.print(F("led = off\r\n"));
  }
  return(ok); 
}

// Comando pan...
int command_pan(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfPan;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 250, &temp)){
      config.rfPan = (byte)temp;
      theData.pan_id = config.rfPan;  
      EEPROM.put(DIR_EEPROM_CFG, config);  
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("pan = %ld\r\n"),(long)config.rfPan);
  return(ok);   
}

// Comando read...
int command_read(int argc, char** argv){
  char s[10];
  unsigned long a = millis();  
  
  // Lee sensores...
  cmd.println(F("Wait..."));    
  lee_sensores();
  i2DecimalPoint(theData.battery, 2, s);
  cmd.printf(F("Battery      = %s v\r\n"),s);  
  if((theData.status & 0x01) == 0x01){
    i2DecimalPoint(theData.temperature, 1, s);
    cmd.printf(F("Temperature  = %s ºC\r\n"),s);
    cmd.printf(F("Humidity     = %d %%\r\n"),theData.humidity);
#ifndef BMP280      
    i2DecimalPoint(theData.dew_point, 1, s);    
    cmd.printf(F("Dew point    = %s ºC\r\n"),s);
#endif    
  }else{
    cmd.println(F("SHT21 sensor not present"));
  }  
  if((theData.status & 0x02) == 0x02){
    i2DecimalPoint(theData.temperature, 1, s);
    cmd.printf(F("Temperature  = %s ºC\r\n"),s);
  }else{
    cmd.println(F("DS18B20 sensor not present"));
  }    
  if((theData.status & 0x04) == 0x04){
    cmd.printf(F("CO2          = %d ppm\r\n"),theData.co2);
  }else{
    cmd.println(F("CO2 sensor not present"));
  }    
  if((theData.status & 0x08) == 0x08){
    cmd.printf(F("Light        = %d %%\r\n"),theData.light);
  }else{
    cmd.println(F("LDR sensor not present"));
  }   
#ifdef BMP280  
  if((theData.status & 0x10) == 0x10){
    i2DecimalPoint(theData.pressure, 1, s);
    cmd.printf(F("Pressure     = %s hPa\r\n"),s);
  }else{
    cmd.println(F("Pressure sensor not present"));
  }   
#else
    cmd.println(F("Pressure sensor not present"));
#endif  


  cmd.printf  (F("Spent time   = %d mS\r\n"), millis() - a);
  return(true);  
}

// Comando send...
int command_send(int argc, char** argv){
  unsigned long a = millis();

  cmd.println(F("Read sensors and send data..."));
  
  // Turn on test led. This led shows us how long the node is awake...
  led_test_on();

  // Sensors read...
  lee_sensores();

  // Show node info...
  cmd.printf (F("Pan     = %d\r\n"), config.rfPan);
  if(config.dipsw)
    cmd.printf (F("Node    = %d\r\n"), leeDIPSW());  
  else  
    cmd.printf (F("Node    = %d\r\n"), config.rfNode);
  cmd.printf (F("Power   = %d\r\n"), config.rfPower);  
  cmd.printf (F("Retries = %d\r\n"), config.rfRetries);
  cmd.println(F("Sending..."));
      
  // Envia mensaje al servidor...
  if(send_to_server()){
    cmd.println(F("Send OK"));     
  }else{
    cmd.println(F("Send ERROR!"));  
  }
  cmd.printf (F("Spent time = %d mS\r\n"), millis() - a);

  // Turn of test led...
  led_test_off();
   
  return(true);    
}

// Comando param...
int command_param(int argc, char** argv){
  cmd.printf(F("pow   = %d\r\n"), (long)config.rfPower);  
  cmd.printf(F("stime = %d\r\n"), (long)config.sendTime);
  cmd.printf(F("retri = %d\r\n"), (long)config.rfRetries);
  cmd.printf(F("node  = %d\r\n"), (long)config.rfNode);
  if(config.dipsw)
    cmd.printf(F("dip   = on\r\n"));  
  else
    cmd.printf(F("dip   = off\r\n"));  
  if(config.led)
    cmd.printf(F("led   = on\r\n"));  
  else
    cmd.printf(F("led   = off\r\n"));      
  cmd.printf(F("pan   = %d\r\n"), (long)config.rfPan);
  return(true);
}

// Comando help...
bool command_help(int argc, char** argv){
  cmd.println(F("Command list:"));
  cmd.printCommands();
  return(false);
}




