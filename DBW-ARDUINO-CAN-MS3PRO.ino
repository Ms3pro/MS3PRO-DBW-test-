#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);//CS 
                  //INT3-2

int TrotleTarged; //MS3PRO DBW TABLE TARGET ID260 - 0X104

int sensor01=A1;//PEDAL POSITION 0-1023
int s1, a, b;   //PEDAL POSITION 0-1023
int sensor02=A2;//THROTTLE POSITION 0-1023
int s2, e, f;   //THROTTLE POSITION 0-1023

unsigned int textCounter = 0;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; 

void setup() 
{
SPI.begin(); 
 Serial.begin(115200);
pinMode(5 , OUTPUT);
  

mcp2515.reset();
mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
mcp2515.setNormalMode(); 
}

void loop() 
{
  TrotleTarged = map(TrotleTarged,0 , 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180)
  analogWrite(5,TrotleTarged);  
               // sets the servo position according to the scaled value
  
if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
for (int i = 0; i<canMsg.can_dlc; i++)   { 
}
}
switch (canMsg.can_id) {
case 260:                          // read servo position map from ms3 //MS3PRO DBW TABLE TARGET ID260 - 0X104

TrotleTarged = (float)(word(canMsg.data[0], canMsg.data[1])); 
break;  
}  
  s1=analogRead(sensor01);  //PEDAL POSITION 0-1023
  b=s1;                     //PEDAL POSITION 0-1023
  s2=analogRead(sensor02);  //THROTTLE POSITION 0-1023
  f=s2;                     //THROTTLE POSITION 0-1023
  canMsg.can_id  = 0x100;   //ARDUINO SEND TO MS DBW ID256       
  canMsg.can_dlc = 8;               
  canMsg.data[0]= s1 >> 8;  //PEDAL POSITION 0-1023            
  canMsg.data[1] = b& 0xff; //PEDAL POSITION 0-1023
  canMsg.data[2] = s2 >> 8; //THROTTLE POSITION 0-1023          
  canMsg.data[3] = f & 0xff;//THROTTLE POSITION 0-1023
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg);
 delay(20);

int DBW = digitalRead(5);
 
 Serial.println(TrotleTarged);
 Serial.println(DBW);
 }
