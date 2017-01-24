//#include <SoftwareSerial.h>
#include <SPI.h>
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"

//#define RX 2
//#define TX 3

//SoftwareSerial mySerial(RX, TX, true); //true for inverted signal

byte data1[32];
byte data2[32];
byte ping[] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2};
uint32_t timeGotFirst;
int firstRecieved = 0;

void setup()
{
  Serial.begin(9600);
  
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"sidpc");
  Mirf.payload = 32;
  Mirf.channel = 15;
  Mirf.configRegister(RF_SETUP, 0x27);
  Mirf.config();
  //Mirf.configRegister(RF_PWR, 11); //11(0db),10(-6),01(-12),00(-18) output power
  //Mirf.configRegister(RF_SETUP, (0x00)|(11<<RF_PWR)|(0<<RF_DR_LOW)|(0<<RF_DR_HIGH)); //power 0db, rate 1Mbps
  Mirf.powerUpRx();
}

void loop()
{
  delay(1000);
  if (Serial.available())
  {
    Serial.read();
    //pingStation();
  }
  if(Mirf.dataReady())
  {
    if(firstRecieved == 0)
    {
      Mirf.getData(data1); //Recieve first packet of data
      if(data1[0] == 35) //confirmation for the first data packet
      { 
        firstRecieved = 1;
        timeGotFirst = millis();
      }
    }
    else
    {
      Mirf.getData(data2);
      if(abs(millis() - timeGotFirst) < 2000 && data2[31] == 35)    //both data packets must be less than two second separated plus confirmation that second packet was recieved
      {
        sendDataToPc();    //both packets recieved, now we can send data to pc
      }
      firstRecieved = 0;
    }    
  }
}

void pingStation()
{
  Mirf.setTADDR((byte *)"hiska");
  Mirf.send(ping);    //auto sets to RX mode  
  while(Mirf.isSending())
  {
   //Wait and set to TX mode when finished
  }
  Serial.println("Sent");
  delay(100);
}
void sendDataToPc()
{
  byte dataToSend[64];
  int w;
  for(w = 0; w < 32; w++)
  {
    dataToSend[w] = data1[w];
    dataToSend[32 + w] = data2[w];
  }
  w = 0;
  while(dataToSend[w] != 36)  //where is locate symbol $, simbolizing end of the string to send to pc
  {
    w++;
  }
  for(int i=1; i < (w-4); i++)  //send from the second char, first is garbage
  //w-3 because dont need to send .00 characters for the last number
  {
    Serial.print(char(dataToSend[i]));
  }
  Serial.println(char(dataToSend[w-4]));
  delay(100);
}
