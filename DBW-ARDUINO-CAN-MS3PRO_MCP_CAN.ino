#include <SPI.h>
#include <mcp_can.h>

#define CAN0_INT 3                          
MCP_CAN CAN0(10); 


unsigned int TrottleTarged; //MS3PRO DBW TABLE TARGET ID260 - 0X104

int sensor01=A1;//PEDAL POSITION 0-1023
int s1, a, b;   //PEDAL POSITION 0-1023
int sensor02=A2;//THROTTLE POSITION 0-1023
int s2, e, f;   //THROTTLE POSITION 0-1023

//unsigned int textCounter = 0;
uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; 

void setup() 
{
SPI.begin(); 
 Serial.begin(115200);
pinMode(A0 , OUTPUT);
pinMode(A1 , INPUT);  
pinMode(A2 , INPUT);  
pinMode(CAN0_INT, INPUT);


  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)  
  CAN0.setMode(MCP_NORMAL);                                                 
}

void loop() 
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

}
switch (rxId) { 
case 260:                          // read servo position map from ms3 //MS3PRO DBW TABLE TARGET ID260 - 0X104

TrottleTarged = (float)(word(rxBuf[0], rxBuf[1])); 
break;  
}  
  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
  if(sndStat == CAN_OK){
  s1=analogRead(sensor01);  //PEDAL POSITION 0-1023
  b=s1;                     //PEDAL POSITION 0-1023
  s2=analogRead(sensor02);  //THROTTLE POSITION 0-1023
  f=s2;                     //THROTTLE POSITION 0-1023
         
            
  data[0]= s1 >> 8;  //PEDAL POSITION 0-1023            
  data[1] = b& 0xff; //PEDAL POSITION 0-1023
  data[2] = s2 >> 8; //THROTTLE POSITION 0-1023          
  data[3] = f & 0xff;//THROTTLE POSITION 0-1023
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
  CAN0.sendMsgBuf(0x100, 0, 8, data); //ARDUINO SEND TO MS DBW ID256  
 
  TrottleTarged = map(TrottleTarged,0 , 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180)
  digitalWrite(A0,TrottleTarged);  
               // sets the servo position according to the scaled value
int DBW = analogRead(A0);
 
 //Serial.println(TrottleTarged);
 Serial.println(DBW);
} 
 }
