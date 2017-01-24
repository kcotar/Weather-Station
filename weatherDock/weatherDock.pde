
#include "EtherShield.h"
#include "LCD12864RSPI.h" 
#include "avr/pgmspace.h"
#include "font.h";
#define FONT_HEIGHT 9
#define AR_SIZE( a ) sizeof( a ) / sizeof( a[0] )
#include "NewSoftSerial.h"
NewSoftSerial mySerial(0, 1); // RX, TX
//NewSoftSerial mySerial(0, 1, true); // this device uses inverted signaling

static uint8_t mymac[6] = {0x52,0x55,0x58,0x10,0x01,0x25};
static uint8_t myip[4] = {192,168,1,142};
// Default gateway. The ip address of your DSL router. It can be set to the same as
// websrvip the case where there is no default GW to access the
// web server (=web server is on the same lan as this host)
static uint8_t gwip[4] = {192,168,1,2};

#define PORT 80   // HTTP
#define T1 4    //tipke
#define T2 5
#define T3 6
#define T4 7
#define LCDback 2

// the etherShield library does not really support sending additional info in a get request
// here we fudge it in the host field to add the API key
// Http header is
#define HOSTNAME "www.kcotar.org"      // API key
static uint8_t websrvip[4] = { 0,0,0,0 };	// Get ip by DNS call
#define WEBSERVER_VHOST "www.kcotar.org"
#define HTTPPATH "/weather/now2.txt"      // Set your own feed ID here

static uint8_t resend=0;
static int8_t dns_state=0;

EtherShield es=EtherShield();

#define BUFFER_SIZE 400
static uint8_t buf[BUFFER_SIZE+1];

unsigned char screenBuffer[1024];
uint32_t requestTime=20000;
char timeC[7], timeCprev[7], tempC[7], humC[6], windC[6], skytC[7], cloudC[6], pressC[7], tempText[19];
uint8_t timeN,tempN,humN,windN,skytN,cloudN,pressN;
float temp[5],hum[5],wind[5],skyt[5],cloud[5],press[5];
uint8_t screenNumber = 1,dataAvailable = 0,izbor = 1, izborA = 1, editA = 0;
char alarmH = 6, alarmMin = 0, alarmStatus = 0;

//int freeRam () {
//  extern int __heap_start, *__brkval; 
//  int v; 
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
//}

void showScreen(uint8_t screen)
{
  uint8_t showPicture;
  char buf1[7],buf2[7];
  LCDClearScreen(false);  
  if(screen == 1 || screen == 3)
  {
    strcpy_P(tempText, PSTR("Temper:"));
    LCDPrintString(1, 0, tempText , false);
    strcpy_P(tempText, PSTR("Vlaga:"));
    LCDPrintString(2, 0, tempText , false);
    strcpy_P(tempText, PSTR("Veter:"));
    LCDPrintString(3, 0, tempText , false);
    strcpy_P(tempText, PSTR("Pritisk:"));
    LCDPrintString(4, 0, tempText , false);
    strcpy_P(tempText, PSTR("Oblaki:"));
    LCDPrintString(5, 0, tempText , false);
    strcpy_P(tempText, PSTR("Nebo:"));
    LCDPrintString(6, 0, tempText , false);  
  }  
  if(screen == 1)
  {
    strcpy_P(tempText, PSTR("Ura:"));
    LCDPrintString(0, 0, tempText, false);       
    LCDPrintString(0, 9, timeC, false);
    
    dtostrf(temp[0],4,1,buf1);
    LCDPrintString(1, 10, buf1, false);
    LCDPlaceCharacter(1, 15, drawSign((int)temp[4]));
    dtostrf(hum[0],4,0,buf1);
    LCDPrintString(2, 10, buf1, false);
    LCDPlaceCharacter(2, 15, drawSign((int)hum[4]));
    dtostrf(wind[0],4,1,buf1);
    LCDPrintString(3, 10, buf1, false);
    LCDPlaceCharacter(3, 15, drawSign((int)wind[4]));
    dtostrf(press[0],4,1,buf1);
    LCDPrintString(4, 9, buf1, false);
    LCDPlaceCharacter(4, 15, drawSign((int)press[4]));
    dtostrf(cloud[0],4,0,buf1);
    LCDPrintString(5, 10, buf1, false);
    LCDPlaceCharacter(5, 15, drawSign((int)cloud[4]));
    dtostrf(skyt[0],4,1,buf1);
    LCDPrintString(6, 10, buf1, false);
    LCDPlaceCharacter(6, 15, drawSign((int)skyt[4]));
    switch((int)cloud[0])
    {
      case 0 : drawPicture(1,14,1); break;
      case 100 : drawPicture(1,14,3); break;
      default : drawPicture(1,14,2);
    }  
    
  }
  if(screen == 3)
  { 
    strcpy_P(tempText, PSTR("max"));
    LCDPrintString(0, 11, tempText, false);
    strcpy_P(tempText, PSTR("min"));
    LCDPrintString(0, 17, tempText, false);
    dtostrf(temp[1],4,1,buf1);
    dtostrf(temp[2],4,1,buf2);
    LCDPrintString(1, 10, buf1, false);
    LCDPrintString(1, 16, buf2, false);
    dtostrf(hum[1],4,0,buf1);
    dtostrf(hum[2],4,0,buf2);
    LCDPrintString(2, 10, buf1, false);
    LCDPrintString(2, 16, buf2, false);
    dtostrf(wind[1],4,1,buf1);
    dtostrf(wind[2],4,1,buf2);
    LCDPrintString(3, 10, buf1, false);
    LCDPrintString(3, 16, buf2, false);
    dtostrf(press[1],5,1,buf1);
    dtostrf(press[2],5,1,buf2);
    LCDPrintString(4, 9, buf1, false);
    LCDPrintString(4, 15, buf2, false);
    dtostrf(cloud[1],4,0,buf1);
    dtostrf(cloud[2],4,0,buf2);
    LCDPrintString(5, 10, buf1, false);
    LCDPrintString(5, 16, buf2, false);
    dtostrf(skyt[1],4,1,buf1);
    dtostrf(skyt[2],4,1,buf2);
    LCDPrintString(6, 10, buf1, false);
    LCDPrintString(6, 16, buf2, false);    
  } 
  if(screen == 2)
  {
    strcpy_P(tempText, PSTR("Rosisce:"));
    LCDPrintString(1, 0, tempText , false);
    strcpy_P(tempText, PSTR("Obcutek:"));
    LCDPrintString(2, 0, tempText , false);
    strcpy_P(tempText, PSTR("HumInd:"));
    LCDPrintString(3, 0, tempText , false);
    
    dtostrf(temp[0]-(100-hum[0])/5. ,4,1,buf1);   //all formulas are from Wikipedia
    LCDPrintString(1, 10, buf1, false);
    dtostrf(13.12 + 0.6215*temp[0] + pow(wind[0],0.16)*(-11.37 + 0.3965*temp[0]) ,4,1,buf1);
    LCDPrintString(2, 10, buf1, false);
//    dtostrf( ,4,1,buf1);
//    LCDPrintString(3, 10, buf1, false);
  } 
  if(screen == 4)    //Screen for changing relay postions
  {
    drawScreen4(false);
  }
  
  if(screen == 5)  //Change alarm settings
  { 
    strcpy_P(tempText, PSTR("NASTAVITEV BUDILKE"));
    LCDPrintString(1, 1, tempText , false);
    strcpy_P(tempText, PSTR("URA:"));
    LCDPrintString(3, 1, tempText , false);
    drawScreen5(false);
  } 
  
  if(screen == 6)    //Reset data screen
  {
    strcpy_P(tempText, PSTR("ALI RES ZELIS"));
    LCDPrintString(2, 1, tempText , false);
    strcpy_P(tempText, PSTR("PONASTAVITI PODATKE"));
    LCDPrintString(3, 1, tempText , false);
    strcpy_P(tempText, PSTR("DA  NE"));
    LCDPrintString(6, 4, tempText , false);
  }
  //dtostrf(freeRam(),4,0,buf1);
  //LCDPrintString(6, 17, buf1, false);
  LCDA.DrawFullScreen(screenBuffer);  
}

void drawScreen4(boolean cond)
{
  strcpy_P(tempText, PSTR(" Luc miza "));
  if(izbor == 1)LCDPrintNegString(1, 1, tempText , cond);
  else LCDPrintString(1, 1, tempText , cond);
  strcpy_P(tempText, PSTR(" Luc strop "));
  if(izbor == 2)LCDPrintNegString(2, 1, tempText , cond);
  else LCDPrintString(2, 1, tempText , cond);
  strcpy_P(tempText, PSTR(" Luc postla "));
  if(izbor == 3)LCDPrintNegString(3, 1, tempText , cond);
  else LCDPrintString(3, 1, tempText , cond);  
}

void drawScreen5(boolean cond)
{
  char buf1[4];
  
  dtostrf((float)alarmH,2,0,buf1);
  if(izborA == 1 && editA == 1) LCDPrintNegString(3, 8, buf1, cond);
  else LCDPrintString(3, 8, buf1, cond);  
  dtostrf((float)alarmMin,2,0,buf1);
  if(izborA == 2 && editA == 1) LCDPrintNegString(3, 11, buf1, cond);
  else LCDPrintString(3, 11, buf1, cond);
  if ((int)alarmStatus == 0) strcpy_P(tempText, PSTR("Alarm OFF"));
  else strcpy_P(tempText, PSTR("Alarm ON "));
  if(izborA == 3 && editA == 1) LCDPrintNegString(5, 1, tempText , cond); 
  else LCDPrintString(5, 1, tempText , cond);
}

void browserresult_callback(uint8_t statuscode,uint16_t datapos)
{  
  int starPos=datapos+100;
  int i,m;
  int j=0,hash[2];
  int k=0,vejice[6];  
  if (datapos != 0 && statuscode==0)
  {
                  //get data postions in buf array
    for(i=0;i<250;i++)
    {
      if(buf[starPos+i]==0x23)    //ascii for #
      {
        hash[j]=starPos+i;
        j++;
      }
      if(buf[starPos+i]==0x2C)    //ascii for ,
      {
        vejice[k]=starPos+i;
        k++; 
      }
    }
    if(j==2&&k==6)
    {   
      //all data are recieved
    }
    else 
    {
      return;
    }
    
    timeN=vejice[0]-hash[0]-1;
    tempN=vejice[1]-vejice[0]-1;
    humN=vejice[2]-vejice[1]-1;
    windN=vejice[5]-vejice[4]-1;
    skytN=vejice[4]-vejice[3]-1;
    cloudN=hash[1]-vejice[5]-1;
    pressN=vejice[3]-vejice[2]-1;
                              //break string into separate values
    memcpy(timeC,&buf[hash[0]+1],timeN);
    timeC[timeN+1]='\0';
    memcpy(tempC,&buf[vejice[0]+1],tempN);
    tempC[tempN+1]='\0';
    memcpy(humC,&buf[vejice[1]+1],humN);
    humC[humN+1]='\0';
    memcpy(windC,&buf[vejice[4]+1],windN);
    windC[windN+1]='\0';
    memcpy(skytC,&buf[vejice[3]+1],skytN);
    skytC[skytN+1]='\0';
    memcpy(cloudC,&buf[vejice[5]+1],cloudN);
    cloudC[cloudN+1]='\0';
    memcpy(pressC,&buf[vejice[2]+1],pressN);
    pressC[pressN+1]='\0';
                              //convert to float
    temp[0]=atof(tempC);
    hum[0]=atof(humC);
    wind[0]=atof(windC);
    skyt[0]=atof(skytC);
    cloud[0]=atof(cloudC);
    press[0]=atof(pressC);
    if(hum[0]>100)hum[0]=100;
    if(cloud[0]>100)cloud[0]=100;
    
    if(dataAvailable==0)      //data recieved for the first time
    {      
      strcpy(timeCprev,timeC);
      resetData();   
      dataAvailable=1;
      requestTime=150000;
    }
    else                       //not the first time that data are recieved                   
    {  
      if(memcmp(timeC,timeCprev,5)==0)
      {
        //data recieved are not newer
      }  
      else 
      { 
        strcpy(timeCprev,timeC);
                          //compare if the current data are bigger or smaller than any previously recieved
        temp[1]=Max(temp[1],temp[0]);
        hum[1]=Max(hum[1],hum[0]);
        wind[1]=Max(wind[1],wind[0]);
        skyt[1]=Max(skyt[1],skyt[0]);
        cloud[1]=Max(cloud[1],cloud[0]);
        press[1]=Max(press[1],press[0]);
        
        temp[2]=Min(temp[2],temp[0]);
        hum[2]=Min(hum[2],hum[0]);
        wind[2]=Min(wind[2],wind[0]);
        skyt[2]=Min(skyt[2],skyt[0]);
        cloud[2]=Min(cloud[2],cloud[0]);
        press[2]=Min(press[2],press[0]);
                                        //are data risinf falling or are they steady
        temp[4]=MoreLessEqual(temp[3],temp[0]);
        hum[4]=MoreLessEqual(hum[3],hum[0]);
        wind[4]=MoreLessEqual(wind[3],wind[0]);
        skyt[4]=MoreLessEqual(skyt[3],skyt[0]);
        cloud[4]=MoreLessEqual(cloud[3],cloud[0]);
        press[4]=MoreLessEqual(press[3],press[0]);
                            
        temp[3]=temp[0];
        hum[3]=hum[0];
        wind[3]=wind[0];
        skyt[3]=skyt[0];
        cloud[3]=cloud[0];
        press[3]=press[0];
      }
    }
    if(screenNumber <= 3)
    {
      screenNumber = 1;
      showScreen(screenNumber);
    }
  }
}
float Max(float x, float y)
{
 if(x>y)return x;
 else return y; 
}
float Min(float x, float y)
{
 if(x<y)return x;
 else return y; 
}
int MoreLessEqual(float x, float y)
{
  if(y>x)  return 1;    // risssing
  if(y<x)  return 2;    // falling
  if(y==x) return 3;    // equal
}
int drawSign(uint8_t z)
{
  switch(z)
  {
    case 1: return 0x0a; break;
    case 2: return 0x0b; break;
    case 3: return 0x2d; break;
  } 
}
void resetData()
{
  uint8_t m;
  for(m=1;m<4;m++)        //fill the array with current data
  {
    temp[m]=temp[0];
    hum[m]=hum[0];
    wind[m]=wind[0];
    skyt[m]=skyt[0];
    cloud[m]=cloud[0];
    press[m]=press[0]; 
  }
  //set all data changes to 3 == equal
  temp[4]=3;
  hum[4]=3;
  wind[4]=3;
  skyt[4]=3;
  cloud[4]=3;
  press[4]=3; 
  
}
void drawPicture(uint8_t row, uint8_t byteNumber, uint8_t picture)
{
  //row from 0 to 64
  //byteNumber from 0 to 15
  if(row>47||byteNumber>14)return;  //picture outside of screen limits
  int i,j;
  for(i=0;i<16;i++)
  {
    for(j=0;j<2;j++)
    {
      screenBuffer[16*(i+row)+byteNumber+j]=pgm_read_byte(&sun[i][j+(picture-1)*2]);
    }
  }
}

void ScreenUp()
{
 if(screenNumber < 5 && editA == 0)
 {
    screenNumber += 1;
    showScreen(screenNumber);
 }  
}

void ScreenDown()
{
 if(screenNumber > 1 && editA == 0)
 {
    screenNumber -= 1;
    showScreen(screenNumber);
 } 
}

void SendCommamnd()
{
  switch(izbor)
  {
     case(1): mySerial.print('m'); break;
     case(2): mySerial.print('s'); break;
     case(3): mySerial.print('p'); break;
  }
}

void SendAlarmSettings()
{
  mySerial.print('i');
  delay(5);
  mySerial.print(alarmH);
  delay(30); 
  mySerial.print('j');
  delay(5);
  mySerial.print(alarmMin);
  delay(30);
  mySerial.print('k');
  delay(5);
  mySerial.print(alarmStatus); 
}

void setup(){
  LCDA.Initialise();
  delay(10);
  LCDClearScreen(true);
  pinMode(T1,INPUT);
  pinMode(T2,INPUT);
  pinMode(T3,INPUT);
  pinMode(T4,INPUT);
  pinMode(LCDback,OUTPUT);      //lcd backlight
  digitalWrite(LCDback,LOW);
  mySerial.begin(9600);
    
  /*initialize enc28j60*/
  es.ES_enc28j60Init(mymac);

  //init the ethernet/ip layer:
  es.ES_init_ip_arp_udp_tcp(mymac, myip, PORT);

  // init the web client:
  es.ES_client_set_gwip(gwip);  // e.g internal IP of dsl router
}

void loop()
{
  static uint32_t timetosend, timeNow, lightOn=0;
  uint16_t dat_p;
  int sec = 0;
  long lastDnsRequest = 0L;
  int plen = 0;
  delay(50);
  mySerial.print('y');
  delay(20);
  alarmH=mySerial.read();
  mySerial.print('x');
  delay(20);
  alarmMin=mySerial.read();
  mySerial.print('z');
  delay(20);
  alarmStatus=mySerial.read();
  strcpy_P(tempText, PSTR("Pocakaj nalagam"));  //copy string from flash
  LCDPrintString(0, 0, tempText , true);
  dns_state=0;
 
  while(1) {
    // handle ping and wait for a tcp packet - calling this routine powers the sending and receiving of data
    plen = es.ES_enc28j60PacketReceive(BUFFER_SIZE, buf);
    dat_p=es.ES_packetloop_icmp_tcp(buf,plen);
    if( plen > 0 ) {
      // We have a packet
      // Check if IP data
      if (dat_p == 0) {
        if (es.ES_client_waiting_gw() ){
          // No ARP received for gateway
          continue;
        }
        // It has IP data
        if (dns_state==0){
          sec=0;
          dns_state=1;
          lastDnsRequest = millis();
          es.ES_dnslkup_request(buf,(uint8_t*)WEBSERVER_VHOST);
          continue;
          
        }
        if (dns_state==1 && es.ES_udp_client_check_for_dns_answer( buf, plen ) ){
          dns_state=2;
          es.ES_client_set_wwwip(es.ES_dnslkup_getip());
        }
        if (dns_state!=2){
          // retry every minute if dns-lookup failed:
          if (millis() > (lastDnsRequest + 60000L) ){
            dns_state=0;
            lastDnsRequest = millis();
          }
          // don't try to use web client before
          // we have a result of dns-lookup
          strcpy_P(tempText, PSTR("Povezujem"));
          LCDPrintString(1, 0, tempText, true);
          continue;
        }
      } else {
        if (dns_state==1 && es.ES_udp_client_check_for_dns_answer( buf, plen ) ){
          dns_state=2;
          es.ES_client_set_wwwip(es.ES_dnslkup_getip());
        }
      }
    }
    // If we have IP address for server and it's time then request data
    timeNow=millis();
    if(  dns_state == 2  &&  abs(timeNow - timetosend) > requestTime  &&  screenNumber!=6 )  // every xy seconds
    {
      //requestTime=60000;
      timetosend = millis();
      // note the use of PSTR - this puts the string into code space and is compulsory in this call
      // second parameter is a variable string to append to HTTPPATH, this string is NOT a PSTR
      es.ES_client_browse_url(PSTR(HTTPPATH), NULL, PSTR(HOSTNAME), &browserresult_callback);
    }
    
    if (mySerial.available() > 0 && dataAvailable == 1) 
    {
      char SerialRecieved = mySerial.read();
      switch(SerialRecieved)
      {
         case('a'): lightOn=timeNow; break;   //display button
         case('b'):        //FWD button                
             ScreenUp();
             break;
         case('c'):       //REW button
             ScreenDown();
             break; 
        case('d'):        //REC button
             if(screenNumber == 4)
             {
                izbor -= 1;
                if(izbor < 1)izbor = 3;
                drawScreen4(true);
             }
             if(screenNumber == 5 && editA == 1)
             {
                switch(izborA)
                {
                  case(1): 
                     alarmH -= 1; 
                     if (alarmH < 0) alarmH = 23; 
                     break;
                  case(2): 
                     alarmMin -= 1; 
                     if (alarmMin < 0) alarmMin = 59; 
                     break;
                  case(3): alarmStatus ^= 1; break;
                }
                drawScreen5(true);
             }
             break;   
        case('e'):        //stop button
             if(screenNumber == 4)
             {
                izbor += 1;
                if(izbor > 3)izbor = 1;
                drawScreen4(true);
             }
             if(screenNumber == 5 && editA == 1)
             {
                switch(izborA)
                {
                  case(1): alarmH = (alarmH + 1) % 24; break;
                  case(2): alarmMin = (alarmMin + 1) % 60; break;
                  case(3): alarmStatus ^= 1; break;
                }
                drawScreen5(true);
             }             
             break;   
        case('f'):     //play/pouse button
             if(screenNumber == 4) SendCommamnd(); 
             if(screenNumber == 5)
             {
               if(editA == 1)
               {
                 switch(izborA)
                 {
                   case(1): izborA = 2; break;
                   case(2): izborA = 3; break;
                   case(3): 
                     izborA = 1;  
                     editA = 0;
                     SendAlarmSettings();
                     break;
                 }
               }
               else editA = 1;
               drawScreen5(true);             
             }
             break;             
      } 
    }
    if(digitalRead(T4))
    {
      lightOn=timeNow;
    }
    
    if(digitalRead(T1))
    {
      //delay(100);   
      if(dataAvailable==1)
      {
        if(screenNumber==6)
        {
          resetData();
        }
        else
        {
          ScreenDown();
        }
      }
    }
    if(digitalRead(T2))
    { 
      //delay(100);
      if(dataAvailable==1)
      {
        if(screenNumber==6)
        {
          screenNumber=1;
          showScreen(screenNumber);
        }
        else
        {
          ScreenUp();
        }
      }
    }   
    if(digitalRead(T3))
    {
      //delay(100);
      if(dataAvailable == 1 && screenNumber < 4 && editA == 0)
      {
        screenNumber=6;
        showScreen(screenNumber);        
      }
    } 
    if(abs(timeNow-lightOn)<12000)
    {
      digitalWrite(LCDback,HIGH);  
    }
    else
    {
      digitalWrite(LCDback,LOW);
    }
  }
}

void LCDClearScreen(bool update) 
{
    for (int i = 0; i < 1024; i++) {
        screenBuffer[i] = 0x00;
    }
    if(update)
    {
      LCDA.DrawFullScreen(screenBuffer);
    }
}

void LCDPrintString(uint8_t row, uint8_t col, char* string, bool update)
{
    int i = 0;
    while (string[i]) {
        LCDPlaceCharacter(row, col, string[i]);
        i++;
        col++;
    }
    
    /* Redraw the row that changed */
    if (update) {
        LCDA.DrawScreenRow(screenBuffer, (row * FONT_HEIGHT) + 1);
    }
}


void LCDPrintNegString(uint8_t row, uint8_t col, char* string, bool update)
{
    int i = 0;
    while (string[i]) {
        LCDPlaceNegCharacter(row, col, string[i]);
        i++;
        col++;
    }
    
    /* Redraw the row that changed */
    if (update) {
        LCDA.DrawScreenRow(screenBuffer, (row * FONT_HEIGHT) + 1);
    }
}

void LCDPlaceCharacter(uint8_t row, uint8_t col, char character)
{
    if (row > 6 || col >= 21) {
        /* Off the bottom or side */
        return;
    }
 
    if (pgm_read_word(&fontLookup[(int)character]) == 0 && (int)character != ' ') {
        /* No character in font */
        return;
    }
    
    /* What bit in the start character does this start? */
    int startBit = (col * 6) % 8;        
    
    /* Add all 10 character rows */
    for (int i = 0; i < FONT_HEIGHT; i++) {
        /* Pixel are in the lower 5 bits */
        
        int pixels = pgm_read_byte(&fontPixel[pgm_read_word(&fontLookup[(int)character])][i]);
        /* Work out what byte this row starts in  */
        int startByte = (i + (row * FONT_HEIGHT) + 1) * 16;
        if (startBit <= 3) {
            /* All within one byte */
            int bits = pixels << 3 - startBit;
            int mask = 0x3F << 3 - startBit - 1;
            screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
        } else {
            /* Accross two bytes */
            int bits = pixels >> startBit - 3;
            int mask = 0x1F >> startBit - 3;
            screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
            
            bits = pixels << (11 - startBit);
            mask = 0x3F << (11 - startBit - 1);
            screenBuffer[startByte + ((col * 6) / 8) + 1] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8) + 1] |= bits; // Add character
        }
    }
}

void LCDPlaceNegCharacter(uint8_t row, uint8_t col, char character)
{
    if (row > 6) {
        /* Off the bottom or side */
        return;
    }
 
    if (pgm_read_word(&fontLookup[(int)character]) == 0 && (int)character != ' ') {
        /* No character in font */
        return;
    }
    
    /* What bit in the start character does this start? */
    int startBit = (col * 6) % 8;        
    
    /* Add all 10 character rows */
    for (int i = 0; i < FONT_HEIGHT; i++) {
        /* Pixel are in the lower 5 bits */
        int pixels = 0x1F & ~pgm_read_byte(&fontPixel[pgm_read_word(&fontLookup[(int)character])][i]);
        
        /* Work out what byte this row starts in  */
        int startByte = (i + (row * FONT_HEIGHT) + 1) * 16;
        if (startBit <= 3) {
            /* All within one byte */
            int bits = pixels << 3 - startBit;
            int mask = 0x1F << 3 - startBit;
            int mask2 = 0x01 << 3 - startBit - 1;
            screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8)] |= mask2; // Black between chars
            screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
        } else {
            /* Accross two bytes */
            int bits = pixels >> startBit - 3;
            int mask = 0x1F >> startBit - 3;
            screenBuffer[startByte + ((col * 6) / 8)] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8)] |= bits; // Add character
            
            bits = pixels << (11 - startBit);
            mask = 0x1F << (11 - startBit);
            int mask2 = 0x01 << (11 - startBit - 1);
            screenBuffer[startByte + ((col * 6) / 8) + 1] &= ~mask; // Clear space
            screenBuffer[startByte + ((col * 6) / 8) + 1] |= mask2;
            screenBuffer[startByte + ((col * 6) / 8) + 1] |= bits; // Add character
        }
    }
}
