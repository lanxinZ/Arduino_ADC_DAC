/*
 * change log: 
 * 1). remove code for reseting PSU_DAC in setup(), because setup() will run as
 * the serial monitor opens. Which means each time the python script runs, setup()
 * runs again.
 * 2). add case 5 for reseting PSU_DAC 
 */

#include <Wire.h>

/* Device address */
#define DAC01_ADD 0b1001100
#define DAC02_ADD 0b1001101
#define DAC_PSU_ADD 0b1001011
#define ADC_ADD 0b1001000

/* ADC address pointer register */
#define CONV_REG    0x00 // Conversion Register
#define CONFIG_REG  0x01 // Configuration Register
#define LO_THRESH_REG   0x02 // Low Threshold Register
#define HI_THRESH_REG   0x03 // High Threshold Register

#define ADC_FSR 2
#if (defined(ADC_FSR) && ADC_FSR == 1) 
  /*single shot mode, LSB=2mV (FSR 4.096V)*/
  #define config_COMP_0_GND   (0x4000 | 0b0000001110000011) 
  #define config_COMP_1_GND   (0x5000 | 0b0000001110000011)
  #define config_COMP_2_GND   (0x6000 | 0b0000001110000011)
  #define config_COMP_3_GND   (0x7000 | 0b0000001110000011)
  #define ADC_LSB ((int)2);
#elif (defined(ADC_FSR) && ADC_FSR == 2)
   /*single shot mode, LSB=3mV (FSR 6.144V)*/
  #define config_COMP_0_GND   (0x4000 | 0b0000000110000011) 
  #define config_COMP_1_GND   (0x5000 | 0b0000000110000011)
  #define config_COMP_2_GND   (0x6000 | 0b0000000110000011)
  #define config_COMP_3_GND   (0x7000 | 0b0000000110000011)
  #define ADC_LSB ((int)3);
#else
  /* ADC config register: single shot mode, other fields are default, besides MUX*/
  #define config_COMP_0_GND   (0x4000 | 0b0000010110000011) 
  #define config_COMP_1_GND   (0x5000 | 0b0000010110000011)
  #define config_COMP_2_GND   (0x6000 | 0b0000010110000011)
  #define config_COMP_3_GND   (0x7000 | 0b0000010110000011)
  #define ADC_LSB ((int)1);
#endif

/* global variables - input from Python*/
int buffer_size = 7;
String str_channel = "00";
String str_value = "0250";
volatile unsigned int i_channel = 1;
volatile unsigned int i_value = 250;
/* global variables - adc result (measured value)*/
volatile int PV_mV = 0; 
volatile int PI_mA = 0;


void setup() {
  Serial.begin(9600); 
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);
}

void loop() {
  char var_buff[buffer_size];
  if(Serial.available()>6){
    input_handle(var_buff, buffer_size);
  }
  else{
    var_buff[0] = 'f';
  }
  Serial.println(var_buff[0]);
  
  delay(200);
  
  switch (var_buff[0]){
    case '0':
      resetAll();
      Serial.print("Case 0 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){}     //清空arduino serial 缓存
      break;
    case '1':
      setV_channel(i_channel, i_value);
      Serial.print("Case 1 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){}
      break;
    case '2':
      setPV_channel(i_channel, i_value);
      Serial.print("Case 2 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){} 
      break;
    case '3':
      getPV_channel(i_channel);
      Serial.print("Case 3 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){}
      break;
    case '4':
      getI_channel(i_channel);
      Serial.print("Case 4 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){}
      break;
    case '5':
      softreset_DAC(DAC_PSU_ADD);
      enableInternRef(DAC_PSU_ADD);
      setPV_channel(1, 2500);
      setPV_channel(2, 2500);
      Serial.print("Case 5 finished.");
      Serial.print(" \n");
      delay(500);
      while(Serial.read()>=0){}
      break;
    default:
      Serial.println("Case default finished.");
      break; 
  }
  
  delay(200);
  
}

void input_handle(char* var_buff, int buffer_size){

    Serial.readBytes(var_buff, buffer_size);
    str_channel = var_buff[1];
    str_channel += var_buff[2];
    
    str_value = var_buff[3];
    for(int i=4;i<7;i++){
      str_value += var_buff[i];
    }
        
    i_channel = str_channel.toInt();
    i_value = str_value.toInt();
    int temp = i_channel + i_value;
    Serial.print("i_channel: ");
    Serial.print(i_channel);
    Serial.print("\t i_value: ");
    Serial.print(i_value);
    Serial.print(" \n");

    while(Serial.read()>=0){}
}

void resetAll(){
  softreset_DAC(DAC01_ADD);
  enableInternRef(DAC01_ADD);
  softreset_DAC(DAC02_ADD);
  enableInternRef(DAC02_ADD);
  softreset_ADC();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("resetAll works.");
  Serial.print(" \n");
}

void setV_channel(unsigned int dac_channel, unsigned int dac_value){
  Serial.print("dac_channel: \t");
  Serial.print(dac_channel);
  Serial.print("\t dac_value: \t");
  Serial.println(dac_value);

  const float fullrange_dec = 5.000;
  const float fullrange_bin = 4.096;
  float factor = (float)dac_value * fullrange_bin / fullrange_dec;
  Serial.println(factor);
  
  // write to Data Buffer X and update Channel X Output
  unsigned char _command = 0x00;
  unsigned int dac_in = (unsigned int)(round(factor));  //dac_in: decimal equivalent of the binary code
  Serial.println(dac_in);
  unsigned int x = (unsigned int) (dac_in << 4); 
  Serial.println(x);
  unsigned char msdb = (unsigned char)(x >> 8);
  Serial.println(msdb, BIN);
  unsigned char lsdb  = x & 0xFF;
  Serial.println(lsdb, BIN);
  
  if(dac_channel<9){ //DAC1: 1~8
    _command = _command + (dac_channel-1);
    transmit_DAC(DAC01_ADD, _command, msdb, lsdb);
  }
  else if(dac_channel<19){ //DAC2: 9~16
    _command = _command + (dac_channel-8-1);
    transmit_DAC(DAC02_ADD, _command, msdb, lsdb);
  }
  else{ //DAC_PSU: 21,22
    _command = _command + (dac_channel-20-1);
    transmit_DAC(DAC_PSU_ADD, _command, msdb, lsdb);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("_command: \t");
  Serial.print(_command, BIN);
  Serial.print("\t msdb: \t");
  Serial.print(msdb, BIN);
  Serial.print("\t lsdb: \t");
  Serial.print(lsdb, BIN);
  Serial.print("\n setV_channel works.");
  Serial.print(" \n");
}

void setPV_channel(unsigned int dac_channel, unsigned int pvout_value){
  Serial.print("pvout_value: \t");
  Serial.println(pvout_value);
  unsigned char _command = 0x00 + (dac_channel-1);
  volatile unsigned int dac_out = 4800 - pvout_value;  //dac_out: [mV]
  Serial.print("first dac_out: \t");
  Serial.println(dac_out);

  /*
  unsigned int dac_channel_dummy = dac_channel+20; //to tell this is the PSU DAC
  unsigned int adc_channel = (dac_channel ==1)? 1 : 2; 
  Serial.print("adc_channel: \t");
  Serial.println(adc_channel);
  setV_channel(dac_channel_dummy, 912); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 1012); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 1312); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 1512); //dac_out must greater than 1500mV, corresponding PVout(max) = 4155mV
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 1812); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 2812); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 3812); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 4100); 
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  setV_channel(dac_channel_dummy, 4212); //dac_out must less than 4200mV, corresponding PVout(min) = 1440mV
  delay(300);
  getPV_channel(adc_channel);
  delay(300);
  */
  
  unsigned int dac_channel_dummy = dac_channel+20; //to tell this is the PSU DAC
  unsigned int adc_channel = (dac_channel ==1)? 1 : 2; 
  Serial.print("adc_channel: \t");
  Serial.println(adc_channel);
  float factor = 0.95;
  volatile int delta = 1000;
  int a = 0;
  do{
    setV_channel(dac_channel_dummy, dac_out); 
    delay(400);    
    getPV_channel(adc_channel);
    delay(400);
    delta = pvout_value - PV_mV;
    float fac_delta = factor * delta;
    dac_out = (int)(round(dac_out - fac_delta));
  
    Serial.println("#### start correcting psu-dac ####");
    Serial.print("delta: \t");
    Serial.println(delta);
    Serial.print("dac_out: \t");
    Serial.println(dac_out);

    a = a+1;
    Serial.print("a: \t");
    Serial.println(a);
  }while((abs(delta)>10)&&(a<10));
  
  
  Serial.print("\n setPV_channel works.");
  Serial.print(" \n");
}

void softreset_DAC(unsigned char DAC_ADD){
  transmit_DAC(DAC_ADD, 0b01110000, 0b00000000, 0b00000000);
  Serial.print("DAC reset: \t");
  Serial.print(DAC_ADD);
  Serial.print("\n");
}

void softreset_ADC(){
  Wire.beginTransmission(0); //I2C General Call
  Wire.write(0x06);    
  Wire.endTransmission();
  Serial.print("ADC reset: \t");
  Serial.print(ADC_ADD);
  Serial.print("\n");
}

void transmit_DAC(unsigned char _address, unsigned char _command, unsigned char _msdb, unsigned char _lsdb) {
  Wire.beginTransmission(_address);
  Wire.write(_command);
  Wire.write(_msdb);
  Wire.write(_lsdb);
  Wire.endTransmission();

  Serial.print("DAC Address: \t");
  Serial.println(_address, BIN);
}

void enableInternRef(unsigned char DAC_ADD){
  transmit_DAC(DAC_ADD, 0b10000000, 0b00000000, 0b00010000);
  Serial.print("Internal Reference enabled. DAC address: \t");
  Serial.print(DAC_ADD, BIN);
  Serial.print(" \n");
}

//set ADC
void getPV_channel(unsigned int i_channel){
  if(i_channel == 1){
    ADC_writeRegister(CONFIG_REG, config_COMP_1_GND);
  }
  else if(i_channel == 2){
    ADC_writeRegister(CONFIG_REG, config_COMP_2_GND);
  }
  else{
    Serial.println("wrong selected channel!");
  }
  startSingleMeasurement();
  //int temp = 0; // for testing!!remember to change!!
  while(ADC_isBusy()){
    //temp +=1;
    //if(temp>50){break;}
    Serial.println("ADC is busy.");   
  }
  PV_mV = ADC_calcResult_mV();
  Serial.print("getPV_channel performed. Measured Voltage in mV: \t");
  Serial.print(PV_mV);
  Serial.print(" \n");
}

void getI_channel(unsigned int i_channel){
  if(i_channel == 0){
    ADC_writeRegister(CONFIG_REG, config_COMP_0_GND);
  }
  else if(i_channel == 3){
    ADC_writeRegister(CONFIG_REG, config_COMP_3_GND);
  }
  else{
    Serial.println("wrong selected channel!");
  }
  startSingleMeasurement();
  //int temp = 0; // for testing!!remember to change!!
  while(ADC_isBusy()){
    //temp +=1;
    //if(temp>50){break;}
    Serial.println("ADC is busy."); 
  }
  PI_mA = ADC_calcResult_mV()*10/25;
  Serial.print("getI_channel performed. Measured Current in mA: \t");
  Serial.print(PI_mA);
  Serial.print(" \n");
}

void ADC_writeRegister(uint8_t reg, uint16_t val){ // uint8_t is equivalent to unsigned char
  uint8_t lVal = val & 255;
  uint8_t hVal = val >> 8;
  Wire.beginTransmission(ADC_ADD);
  Wire.write(reg);
  Wire.write(hVal);
  Wire.write(lVal);
  Wire.endTransmission();
  
  Serial.print("ADC write register: \t");
  Serial.print(reg, BIN);
  Serial.print("\t ADC Address: \t");
  Serial.print(ADC_ADD, BIN);
  Serial.print("\t hVal: \t");
  Serial.print(hVal, BIN);
  Serial.print("\t lVal: \t");
  Serial.print(lVal, BIN);
  Serial.print(" \n");
}

uint16_t ADC_readRegister(uint8_t reg){
  uint8_t MSByte = 0, LSByte = 0;
  uint16_t regValue = 0;
  Wire.beginTransmission(ADC_ADD);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADC_ADD,2);
  if(Wire.available()){
    MSByte = Wire.read();
    LSByte = Wire.read();
  }
  regValue = (MSByte<<8) + LSByte;

  Serial.print("ADC read register: \t");
  Serial.print(reg, BIN);
  Serial.print("\t regValue: \t");
  Serial.println(regValue, BIN);
  
  return regValue;
}

int ADC_calcResult_mV(){
  int16_t rawResult = ADC_readRegister(CONV_REG);
  int result_mV = (int)(rawResult >> 4);
  result_mV *= ADC_LSB;
  Serial.print("ADC calculate result in mV: \t");
  Serial.println(result_mV);
  return (result_mV<0)? 0 : result_mV;
}

void startSingleMeasurement(){
  uint16_t currentConfReg = ADC_readRegister(CONFIG_REG);
  currentConfReg |= (1 << 15);
  ADC_writeRegister(CONFIG_REG, currentConfReg);

  Serial.println("Single Measurement started.");
}

bool ADC_isBusy(){
  //0 : Device is currently performing a conversion
  //1 : Device is not currently performing a conversion
  uint16_t currentConfReg = ADC_readRegister(CONFIG_REG);
  return (!(currentConfReg>>15) & 1);
}
