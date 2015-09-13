// Wire Master Reader
#include <Wire.h>
int table[]={0,0,0};
int led1=51;
int led2=52;
int led3=53;

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT);
}

void loop()
{
  Wire.requestFrom(2, 3);    // request 3 bytes from slave device #2

    for(int i=0;i<3;i++)
    {
    int c = Wire.read(); // receive a byte as character

    Serial.print(c);
    table[i]=c;
    Serial.print('\t');   
   
    }
     Serial.print('\n');
       Serial.print(table[0]);
        Serial.print('\t');
       Serial.print(table[1]);
        Serial.print('\t');
       Serial.print(table[2]);
       Serial.print('\n');
   
    if(table[0]>150)
    {
      digitalWrite(led1,HIGH);
    }else{
      digitalWrite(led1,LOW);
    }
   
    if(table[1]>150)
    {
      digitalWrite(led2,HIGH);
    }else{
      digitalWrite(led2,LOW);
    }
    if(table[2]==1)
    {
      digitalWrite(led3,HIGH);
    }else{
      digitalWrite(led3,LOW);
    }   
  delay(500);
}
