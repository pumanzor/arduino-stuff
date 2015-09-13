// Wire Slave Sender

#include <Wire.h>
int table[]={0,0,0};
int lightSensor1=A0;
int lightSensor2=A1;
int button=8;

void setup()
{
  Serial.begin(9600);
  pinMode(lightSensor1,INPUT);
  pinMode(lightSensor2,INPUT);
  pinMode(button,INPUT);
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  table[0]=map(analogRead(lightSensor1),0,1023,0,255);
  table[1]=map(analogRead(lightSensor2),0,1023,0,255);
  if(digitalRead(button)==HIGH)
  {
    table[2]=1;
  }else{
    table[2]=0;
  }
  Serial.print(table[0]);
  Serial.print('\t');
  Serial.print(table[1]);
  Serial.print('\t');
  Serial.print(table[2]);
  Serial.print('\n');
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
//  for(int i=0;i<3;i++)
//  {
  uint8_t Buffer[3];
  Buffer[0]=table[0];
  Buffer[1]=table[1];
  Buffer[2]=table[2];
 
  Wire.write(Buffer,3);
   
}
