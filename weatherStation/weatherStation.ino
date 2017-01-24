#include "Wire.h"          
#include "i2cmaster.h"    //MLX90614ESF sensor library
#include <Adafruit_BMP085.h>       //BMP085 sensor library
#include "PcInt.h"      //an extension to the interrupt support for arduino
#include "avr/power.h"    //for powering down single part of atmega chip
#include "avr/sleep.h"    //sleep functions
#include <SPI.h>
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"
#include <avr/wdt.h>
 
Adafruit_BMP085 bmp;
int humSenPin = 0;    // choose a humidity input pin
int sensorValue;     // value coming from the sensor
float sensorVoltage;      // sensor voltage
float supplyVolt = 3.3; // supply voltage
float zeroVoltage = 0.528; // Define the voltage the sensor returns at 0% humidity.
float calibrationVoltage = 0.0; //compensation voltage for consisten inacuracy
float maxVoltage;    //voltage at 100% humidity
double pressure,humidity,IRtemperature,temperature; 
float trueRH;      // RH % accounting for temperature
int wind=0,rain=0,a=0,b=0;
float windSpeed=0;
int cloudCover;        //percentage of sky covered with clouds

byte data[64];
byte dataRec[32];
//byte stringToSendBlank[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//byte ping[] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2};
int dataPos = 0;
int readyToSend = 1;
 
void setup() 
{
  //Serial.begin(9600);  //BT
  
  bmp.begin();
  i2c_init();  
  PORTC = (1 << PORTC4) | (1 << PORTC5);      //enable pullups for I2C
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(8, INPUT); 
  
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"hiska");
  Mirf.payload = 32;
  Mirf.channel = 15;  
  Mirf.config();
  Mirf.configRegister(RF_SETUP, 0x27);
  //Mirf.configRegister(RF_PWR, 11); //11(0db),10(-6),01(-12),00(-18) output power
  //Mirf.configRegister(RF_SETUP, (0x00)|(11<<RF_PWR)|(0<<RF_DR_LOW)|(0<<RF_DR_HIGH)); //power 0db, rate 1Mbps
  sendData2();
  
  //attachInterrupt(0, incRain, FALLING);    //raingauge interrupt pin
  attachInterrupt(1, incWind, FALLING);      //windmill interrupt pin
  //PCattachInterrupt(4, wakeUp, FALLING);    //PC interrupt pin if using Bluetoothh connection
  //PCattachInterrupt(8, wakeUp, FALLING);    //PC interrupt pin if using NRF24L01 connection
  
  //Watchdog setup
  MCUSR &= ~(1<<WDRF);   
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3;
  WDTCSR |= _BV(WDIE);
}

float relativeHumidity()
{
  trueRH=0;
  maxVoltage=(2.1582 - (0.004426 * temperature));
  for(int i=0;i<10;i++)      //do 10 measurements for better accuracy
  {
    sensorValue = analogRead(humSenPin);
    sensorVoltage= sensorValue/1023.0*supplyVolt;    
    trueRH += ((sensorVoltage + calibrationVoltage - zeroVoltage) / maxVoltage) * 100;
  }
  trueRH=trueRH/10.;      //average
  return trueRH; 
}

float tempIR()
{
  float IRtemp=0;
  for(int j=0;j<3;j++)
  {
    int dev = 0x5A<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
    IRtemp += tempData - 273.15;
  }
  IRtemp=IRtemp/3.;
  return IRtemp;
}

int calculateCloudCover() 
{
  float tempDiff=abs(IRtemperature-temperature);
  if(tempDiff>22)return 0;
  if(tempDiff<=22&&tempDiff>17.75)return 20;
  if(tempDiff<=17.75&&tempDiff>13.5)return 40;
  if(tempDiff<=13.5&&tempDiff>9.25)return 60;
  if(tempDiff<=9.25&&tempDiff>5)return 80;
  if(tempDiff<=5)return 100;  
}

void incWind()
{
  wind++;
  a=1;
}
void incRain()
{
  rain++;
  b=1;
}
void wakeUp()
{
 //empty function to wake up chip, timers and serial.print not working here
}

ISR(WDT_vect)
{
  //empty function to wake up chip, timers when watchdog bites
}

void sendData()    //bt edition
{
  //detachInterrupt(0);        //disabling interrupts before sending data
  detachInterrupt(1);
  //calculate rain            //never
  windSpeed=wind/3.55;
  temperature=bmp.readTemperature();
  humidity=((int)(relativeHumidity()*10))/10.0;
  if(humidity>100)humidity=100;
  pressure=((int)(bmp.readPressure()/10.0))/10.0;    //converting from pascal to mbar
  IRtemperature=((int)(tempIR()*10))/10.0;
  cloudCover=calculateCloudCover();
  if(temperature>100)
  {
     Serial.println(":NULL,NULL,NULL,NULL,NULL,NULL");
  }
  else
  {
    Serial.print(":");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(IRtemperature);  
    Serial.print(",");
    Serial.print(windSpeed);
    Serial.print(",");
    Serial.println(cloudCover);
  }
  delay(100);  //without this sleep function causes serial error / gibberish
  wind=0;
  rain=0;
  //attachInterrupt(0, incRain, FALLING);      //reenabling interrupts after sending is complete
  attachInterrupt(1, incWind, FALLING);
  PCattachInterrupt(4, wakeUp, FALLING);    
}

void sendData2()    //radio edition
{
  detachInterrupt(1);
  Mirf.configRegister(CONFIG, mirf_CONFIG | (1<<PWR_UP));
  delay(100);    //wait some time for propwer power-up of nrf24l
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"hiska");
  Mirf.payload = 32;
  Mirf.channel = 15;  
  Mirf.config();
  Mirf.configRegister(RF_SETUP, 0x27);
  
  windSpeed=wind/3.55*(16.0/30.0);    //compensation for smaller intervals of ~16s instead of ~30s
  temperature=bmp.readTemperature();
  humidity=relativeHumidity();
  if(humidity>100)humidity=100;
  pressure=bmp.readPressure()/100.;    //converting from pascal to mbar
  IRtemperature=tempIR();
  cloudCover=calculateCloudCover();

  byte sender1[32];
  byte sender2[32];
  clearDataArray();
  if(temperature>100)
  {
     //byte toSend[] = "#:NULL,NULL,NULL,NULL,NULL,NULL#";
     data[0] = 35;
     data[1] = 58;
     data[3] = 44;
     data[5] = 44;
     data[7] = 44;
     data[9] = 44;
     data[11] = 44;
     data[13] = 36;
     data[63] = 35;
  }
  else
  {  
    dataPos = 0;
    data[dataPos] = 35;  //#
    dataPos ++;
    data[dataPos] = 58;  //:
    dataPos ++;
    addFloatToDataArray(temperature);
    data[dataPos] = 44;  //,
    dataPos ++;
    addFloatToDataArray(humidity);
    data[dataPos] = 44; 
    dataPos ++;
    addFloatToDataArray((pressure));
    //addFloatToDataArray(0.0);
    data[dataPos] = 44;
    dataPos ++;
    addFloatToDataArray(IRtemperature);
    //addFloatToDataArray(0.0);
    data[dataPos] = 44;
    dataPos ++;
    addFloatToDataArray(windSpeed);
    data[dataPos] = 44;
    dataPos ++;
    addFloatToDataArray(cloudCover);
    //addFloatToDataArray(0.0);
    data[dataPos] = 36; //$
    data[63] = 35;  //#    
  }
  for(int i=0; i<32; i++)
  {
    sender1[i] = data[i];
    sender2[i] = data[32+i];
  } 
  Mirf.setTADDR((byte *)"sidpc");
  Mirf.send(sender1);    //auto sets to RX mode
  while(Mirf.isSending())
  {
   //Wait and set to TX mode when finished
  }
  Mirf.setTADDR((byte *)"sidpc");
  delay(100);
  Mirf.send(sender2);    //auto sets to RX mode
  while(Mirf.isSending())
  {
   //Wait and set to TX mode when finished
  }
  Mirf.configRegister(CONFIG, mirf_CONFIG | (0<<PWR_UP));
  wind=0;
  rain=0;
  attachInterrupt(1, incWind, FALLING);  
}

void addFloatToDataArray(double input)
{
  if(input < 0)
  {
    data[dataPos] = 45;  //-
    dataPos++;
  }
  if(abs(int(input)) >= 100)
  {
     data[dataPos] = abs(int(input / 100)) + 48; 
     dataPos++;
  }
  if(abs(int(input)) >= 10)
  {
     data[dataPos] = abs(int((int(input) % 100) / 10)) + 48; 
     dataPos++;
  }
  if(abs(int(input)) >= 1)
  {
     data[dataPos] = abs(int(input) % 10) + 48;
     dataPos++;
  }
  else
  {
     data[dataPos] = 0 + 48;
     dataPos++;
  }
  data[dataPos] = 46;   //.
  dataPos++;
  data[dataPos] = abs(int(input * 10) % 10) + 48;
  dataPos++;
  data[dataPos] = abs(int(input * 100) % 10) + 48;
  dataPos++;
}

void clearDataArray()
{
  for(int i=0; i<64; i++)
  {
    data[i] = 0;
  } 
}

void loop() 
{  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    if(a==1||b==1)    //interupt was caused by windmill or raingauge
    {
      a=0;
      b=0;
    }
    else            //PC request for data or interupted by internal watchdog 
    {
      //PCdetachInterrupt(4);  //BT
      //sendData();
      if(Mirf.dataReady())    //read and discard any data recieved by nrf24l
      {
        Mirf.getData(dataRec);
      }
      if (readyToSend)
      {
        sendData2();  //radio
        readyToSend = 0;
      }
      else
      {
        readyToSend++;
      }
    }
}
