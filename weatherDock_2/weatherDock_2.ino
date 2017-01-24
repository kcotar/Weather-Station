#include "IRremote.h"
#include <Wire.h>
#include "RTClib.h"

#define IRREC 3
#define SPEAKER 4
#define L_DESK 9
#define L_BED 10
#define L_CEILING 11

IRrecv irrecv(IRREC);
decode_results results;
RTC_DS1307 RTC;

int l_desk, l_bed, l_ceiling;
char alarmH = 5, alarmMin = 58, alarmStatus = 0;
int timeNow = 0, time2 = 0, checkAlarmTime = 30000;


void ShutDownAll()
{
  l_desk = 1, l_bed = 1, l_ceiling = 1;
  digitalWrite(L_DESK,l_desk);
  digitalWrite(L_BED,l_bed);
  digitalWrite(L_CEILING,l_ceiling);
}

void TurnOnAll()
{
  l_desk = 0, l_bed = 0, l_ceiling = 0;
  digitalWrite(L_DESK,l_desk);
  digitalWrite(L_BED,l_bed);
  digitalWrite(L_CEILING,l_ceiling);
}

void CheckAlarm()
{
  DateTime now = RTC.now();
  if(now.hour() == alarmH && now.minute() == alarmMin && alarmStatus)
  {
    int toneStart = int(random(200,500));
    playTone(toneStart + 10, 500);
    playTone(toneStart + 20, 600);
    playTone(toneStart + 30, 700);
    playTone(toneStart + 40, 800);
    playTone(toneStart + 50, 900);
    playTone(toneStart + 60, 1000);
    playTone(toneStart + 70, 900);
    playTone(toneStart + 80, 800);
    playTone(toneStart + 90, 700);
    playTone(toneStart + 100, 600);
    playTone(toneStart + 110, 500);
    delay(500);
    int randL = int(random(9,12));
    switch(randL)
    {
      case 9 : l_desk = 0; digitalWrite(L_DESK,l_desk); break;
      case 10 : l_bed = 0; digitalWrite(L_BED,l_bed); break;
      case 11 : l_ceiling = 0; digitalWrite(L_CEILING,l_ceiling); break;
      case 12 : TurnOnAll(); break;
    }
    time2 += checkAlarmTime;
  }
}

void playTone(long duration, int freq) {
    duration *= 1000;
    int period = (1.0 / freq) * 1000000;
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        digitalWrite(SPEAKER,HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(SPEAKER, LOW);
        delayMicroseconds(period / 2);
        elapsed_time += (period);
    }
}

void ChangeRelayStatus(int RelayPin)
{
  switch(RelayPin)
  {
    case(L_DESK): l_desk ^= 1; digitalWrite(L_DESK,l_desk); break;
    case(L_BED): l_bed ^= 1; digitalWrite(L_BED,l_bed); break;
    case(L_CEILING): l_ceiling ^= 1; digitalWrite(L_CEILING,l_ceiling); break;    
  }
}

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(SPEAKER,OUTPUT);
  digitalWrite(SPEAKER,LOW);
  pinMode(L_DESK,OUTPUT);
  pinMode(L_BED,OUTPUT);
  pinMode(L_CEILING,OUTPUT);
  ShutDownAll();
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime(__DATE__, __TIME__));
}

void loop()
{
  timeNow=millis();
  if(abs(timeNow-time2)>checkAlarmTime)   //check alarm every 30s
  {
    time2=timeNow;
    CheckAlarm();  
  }
  
  if (irrecv.decode(&results)) 
  {
    switch(results.value)
    {
       case 0xA05F807F: ShutDownAll(); break;  //power button	
       case 0xA05FB24D: Serial.print('a'); break; //display button
       case 0xA05FB847: Serial.print('b'); break; //FWD button
       case 0xA05F38C7: Serial.print('c'); break; //REW button
       case 0xA05F906F: ChangeRelayStatus(L_DESK); break; //Num1
       case 0xA05F50AF: ChangeRelayStatus(L_BED); break;  //Num2
       case 0xA05FD02F: ChangeRelayStatus(L_CEILING); break;  //Num3
       case 0xA05FE817: Serial.print('d'); break; //REC button
       case 0xA05F18E7: Serial.print('e'); break; //stop button
       case 0xA05F6897: Serial.print('f'); break; //play/pouse button	
    }
    irrecv.resume(); // Receive the next value
  }
  if(Serial.available())
  {
    char SerialRecieved = Serial.read(); 
    switch(SerialRecieved)
    {
      case('m'): ChangeRelayStatus(L_DESK); break;
      case('s'): ChangeRelayStatus(L_CEILING); break;
      case('p'): ChangeRelayStatus(L_BED); break;
      case('y'): Serial.print(alarmH); break;
      case('x'): Serial.print(alarmMin); break;
      case('z'): Serial.print(alarmStatus); break;
      case('i'): delay(15); alarmH = Serial.read(); break;
      case('j'): delay(15); alarmMin = Serial.read(); break;
      case('k'): delay(15); alarmStatus = Serial.read(); break;
    }
  }  
  
}
