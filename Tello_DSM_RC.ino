/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>



int ledState = LOW;   
 
#define LED_B D2

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

const char* ssid      = "TELLO-C53834";
const char* password  = "";

IPAddress TelloIp(192, 168, 10, 1);
unsigned int TelloPort = 8889;      // local port to listen on

WiFiUDP Udp;

char  cmdCommandPacekt[] = "command"; // command
char  cmdTakeoffPacekt[] = "takeoff"; // takeoff
char  cmdLandPacekt[] = "land";       // land
char  cmdRcPacekt[128];               // RC

unsigned int localUdpPort = 9002;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacekt[] = "Hi there! Got the message :-)";  // a reply string to send back


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


const uint8_t syncArray1[14] = { 1,0,3,2,5,4,7,6,9,8,11,10,13,12};
const uint8_t syncArray2[14] = { 1,0,3,2,7,6,5,4,11,10,13,12,9,8};
  
int bufferIndex=0;

typedef union{
  struct{
    uint16_t aileron;   //A8
    uint16_t aux1;      //A9
    uint16_t elevator;  //A10
    uint16_t gear;      //A11
    uint16_t rudder;    //A12
    uint16_t aux2;      //A13
    uint16_t throttle;  //A14
    uint16_t aux3;      //A15 only for futaba or standard RC
  }
  values;
  byte buffer[16];
  uint16_t standardRCBuffer[8];
}
RadioControl_t;


RadioControl_t rcCommands;

uint8_t rcType,readState,inByte;
boolean detected = false;
boolean newData = false;

#define DSM2 0
#define DSMX 1
#define HEX_ZERO 0x00
long timer;


#define TELLO_TAKEOFF 0
#define TELLO_LAND    1

int TelloState = TELLO_LAND;

int old_a,old_b,old_c,old_d;


#define RC_V 4

void setup()
{
  char strOutput[64];

  pinMode(LED_B, OUTPUT);
  
  Serial1.begin(115200);

  
  Serial.begin(115200);
  
  Serial.println();
  inputString.reserve(32);
  
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    
     if (ledState == LOW){
        ledState = HIGH;  // Note that this switches the LED *off*
        digitalWrite(LED_B, LOW);
     }else{
        ledState = LOW;   // Note that this switches the LED *on*
        digitalWrite(LED_B, HIGH);
    }


  }
  Serial.println(" connected");

  Udp.beginPacket(TelloIp, TelloPort);
  Udp.write(cmdCommandPacekt);
  Udp.endPacket();  
  delay(100);
  Udp.beginPacket(TelloIp, TelloPort);
  Udp.write(cmdCommandPacekt);
  Udp.endPacket();  
    

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  readState = 0;
  Spektrum();
  if (detected == true){
    Serial.println("Detected Spektrum");
    Serial.printf("%02X",rcType);
  }
  old_a = 0; old_b = 0; old_c = 0; old_d = 0;

}



void loop()
{

  DSMXParser();

    if(rcCommands.values.gear < 1500)
    {
      if(TelloState == TELLO_LAND )
      {
          TelloState = TELLO_TAKEOFF;
          Serial.println("TELLO_Takeoff");
          Udp.beginPacket(TelloIp, TelloPort);
          Udp.write(cmdTakeoffPacekt);
          Udp.endPacket();
      }

    }else{
    
      if(TelloState == TELLO_TAKEOFF )
      {
          TelloState = TELLO_LAND;
          Serial.println("TELLO_Land");
          Udp.beginPacket(TelloIp, TelloPort);
          Udp.write(cmdLandPacekt);
          Udp.endPacket();
      }
    }

    if(TelloState == TELLO_TAKEOFF)
    {
      int a,b,c,d;

        a = (int)( 1500 - rcCommands.values.aileron ) /RC_V;
        b = (int)( rcCommands.values.elevator - 1500) /RC_V;
        c = (int)( rcCommands.values.throttle - 1500) /RC_V;
        d = (int)( 1500 - rcCommands.values.rudder ) /RC_V;

        if (abs(c) <20)
        {
          c = 0;  
        }
        if (abs(a) <5)
        {
          a = 0;  
        }  
        if (abs(b) <5)
        {
          b = 0;  
        }  
        if (abs(d) <5)
        {
          d = 0;  
        }  
  
      sprintf(cmdRcPacekt,"rc %d %d %d %d", a,b,c,d);
      Serial.println(cmdRcPacekt);

      if(( old_a == a )&&( old_b == b )&&( old_c == c )&&( old_d == d ))
      {

      }else{
          Serial.println(cmdRcPacekt);
          old_a = a; old_b = b; old_c = c; old_d = d;
          Udp.beginPacket(TelloIp, TelloPort);
          Udp.write(cmdRcPacekt);
          Udp.endPacket();
      }
      
    }

}


void Spektrum(){
 
  timer = millis();
  while (Serial.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }  
  delay(1);
  while (Serial.available() > 0){
    inByte = Serial.read();
    switch(readState){
    case 0:
      if (inByte == 0x03 || 0x21){
        readState = 1;
      }
      if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
        readState = 2;
      }
      break;
    case 1:
      if (inByte == 0xA2 || inByte == 0xB2){
        rcType = DSM2;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    case 2:
      if (inByte == 0xA2){
        rcType = DSMX;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    }

  }
}



void DSMXParser(){

  if (Serial.available() > 14){
    while(Serial.available() > 0){
      inByte = Serial.read();
      switch (readState){
      case 0:
        if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte & 0x07;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      default:
        break;
      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,306,1738)  - 306) * 0.69832 + 1000;
    if (rcCommands.values.aux2 != 0){
      rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,306,1738)  - 306) * 0.69832 + 1000;
    }
  }

}
void DSM2Parser(){

  if (Serial.available() > 14){
    while(Serial.available() > 0){
      inByte = Serial.read();
      switch (readState){
      case 0:
        if (inByte == 0x03 || 0x21){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        if (inByte == 0xB2){
          readState = 3;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      case 3:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray2[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray2[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      default:
        break;

      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,1177,1893)  - 1177) * 1.3908 + 1000;
  }

}

