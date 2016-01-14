#include <AccelStepper.h>
#include <Wire.h>
#include "Kalman.h" 


#define RESTRICT_PITCH 

Kalman kalmanX; 
Kalman kalmanY;

///////////stepeer

AccelStepper stepper1(1, 9, 8);

AccelStepper stepper2(1, 7, 6);
int pos = 2000;
//int cantidad = 7;
int addr = 0;


// Define our three input button pins
#define  LEFT_PIN  4
#define  STOP_PIN  3
#define  RIGHT_PIN 5
#define  CONTROL_PIN 11

#define  SPEED_PIN 0

int velocidad = 1000;
int cantidad = 6;
int aceleracion = 500;



/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
   pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
    pinMode(CONTROL_PIN, INPUT_PULLUP);

  
  
  Serial.begin(115200);
  Wire.begin();
  

  
  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

 stepper1.setMaxSpeed(3000);
  stepper2.setMaxSpeed(3000);
  
    stepper1.setAcceleration(500);
      stepper2.setAcceleration(500); 

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];


#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;


}


/////////////////7

///////////////7







void loop() {
  
  
//////

/////

  
  
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

//Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
  delay(2);
  
  
//////////////////7


if (digitalRead(CONTROL_PIN) == LOW)


{

int valuey = kalAngleX; 
 int valuex = kalAngleY;
 
 
 if (valuey >= -89 && valuey <= -85 )
 
 {stepper2.moveTo(-87 * cantidad);
    stepper2.run();}

else if (valuey >= -84 && valuey <= -80 )
 
 {stepper2.moveTo(-83 * cantidad);
    stepper2.run();}

else if (valuey >= -79 && valuey <= -75 )
 
 {stepper2.moveTo(-77 * cantidad);
    stepper2.run();}

else if (valuey >= -74 && valuey <= -70 )
 
 {stepper2.moveTo(-73 * cantidad);
    stepper2.run();}

else if (valuey >= -69 && valuey <= -65 )
 
 {stepper2.moveTo(-67 * cantidad);
    stepper2.run();}

else if (valuey >= -64 && valuey <= -60 )
 
 {stepper2.moveTo(-63 * cantidad);
    stepper2.run();}

else if (valuey >= -59 && valuey <= -55 )
 
 {stepper2.moveTo(-67 * cantidad);
    stepper2.run();}

else if (valuey >= -54 && valuey <= -50 )
 
 {stepper2.moveTo(-53 * cantidad);
    stepper2.run();}

else if (valuey >= -49 && valuey <= -45 )
 
 {stepper2.moveTo(-47 * cantidad);
    stepper2.run();}
       
else if (valuey >= -44 && valuey <= -40 )
 
 {stepper2.moveTo(-43 * cantidad);
    stepper2.run();}   
    
else if (valuey >= -39 && valuey <= -35 )
 
 {stepper2.moveTo(-37 * cantidad);
    stepper2.run();}    
    
else if (valuey >= -34 && valuey <= -30 )
 
 {stepper2.moveTo(-33 * cantidad);
    stepper2.run();}  

else if (valuey >= -29 && valuey <= -25 )
 
 {stepper2.moveTo(-27 * cantidad);
    stepper2.run();}   
    
else if (valuey >= -24 && valuey <= -20 )
 
 {stepper2.moveTo(-23 * cantidad);
    stepper2.run();}  
    
else if (valuey >= -19 && valuey <= -15 )
 
 {stepper2.moveTo(-17 * cantidad);
    stepper2.run();}  
    
else if (valuey >= -14 && valuey <= -10 )
 
 {stepper2.moveTo(-13 * cantidad);
    stepper2.run();} 
   
else if (valuey >= -9 && valuey <= -5 )
 
 {stepper2.moveTo(-7 * cantidad);
    stepper2.run();}   
    
else if (valuey >= -4 && valuey <= -1 )
 
 {stepper2.moveTo(-3 * cantidad);
    stepper2.run();}  

else if (valuey >= 1 && valuey <= 5 )
 
 {stepper2.moveTo(3 * cantidad);
    stepper2.run();}
    
else if (valuey == 0)
 
 {stepper2.moveTo(0 * cantidad);
    stepper2.run();}   
    
 

else if (valuey >= 6 && valuey <= 10 )
 
 {stepper2.moveTo(8 * cantidad);
    stepper2.run();}

else if (valuey >= 11 && valuey <= 15 )
 
 {stepper2.moveTo(13 * cantidad);
    stepper2.run();}
    
else if (valuey >= 16 && valuey <= 20 )
 
 {stepper2.moveTo(18 * cantidad);
    stepper2.run();}
    
else if (valuey >= 21 && valuey <= 25 )
 
 {stepper2.moveTo(23 * cantidad);
    stepper2.run();}
    
else if (valuey >= 26 && valuey <= 30 )
 
 {stepper2.moveTo(28 * cantidad);
    stepper2.run();}
    
else if (valuey >= 31 && valuey <= 35 )
 
 {stepper2.moveTo(33 * cantidad);
    stepper2.run();}
    
else if (valuey >= 36 && valuey <= 40 )
 
 {stepper2.moveTo(38 * cantidad);
    stepper2.run();}
    
else if (valuey >= 41 && valuey <= 45 )
 
 {stepper2.moveTo(43 * cantidad);
    stepper2.run();}
    
else if (valuey >= 46 && valuey <= 50 )
 
 {stepper2.moveTo(48 * cantidad);
    stepper2.run();}
      
else if (valuey >= 51 && valuey <= 55 )
 
 {stepper2.moveTo(53 * cantidad);
    stepper2.run();}
      
else if (valuey >= 56 && valuey <= 60 )
 
 {stepper2.moveTo(58 * cantidad);
    stepper2.run();}

else if (valuey >= 61 && valuey <= 65 )
 
 {stepper2.moveTo(63 * cantidad);
    stepper2.run();}

else if (valuey >= 66 && valuey <= 70 )
 
 {stepper2.moveTo(68 * cantidad);
    stepper2.run();}

else if (valuey >= 71 && valuey <= 85 )
 
 {stepper2.moveTo(73 * cantidad);
    stepper2.run();}



else if (valuey >= 86 && valuey <= 89 )
 
 {stepper2.moveTo(88 * cantidad);
    stepper2.run();}

//////////////////////////////
/////////////////////////////
/////////////////////////////


else {}


 if (valuex >= -89 && valuex <= -85 )
 
 {stepper1.moveTo(-87 * cantidad);
    stepper1.run();}

else if (valuex >= -84 && valuex <= -80 )
 
 {stepper1.moveTo(-83 * cantidad);
    stepper1.run();}

else if (valuex >= -79 && valuex <= -75 )
 
 {stepper1.moveTo(-77 * cantidad);
    stepper1.run();}

else if (valuex >= -74 && valuex <= -70 )
 
 {stepper1.moveTo(-73 * cantidad);
    stepper1.run();}

else if (valuex >= -69 && valuex <= -65 )
 
 {stepper1.moveTo(-67 * cantidad);
    stepper1.run();}

else if (valuex >= -64 && valuex <= -60 )
 
 {stepper1.moveTo(-63 * cantidad);
    stepper1.run();}

else if (valuex >= -59 && valuex <= -55 )
 
 {stepper1.moveTo(-67 * cantidad);
    stepper1.run();}

else if (valuex >= -54 && valuex <= -50 )
 
 {stepper1.moveTo(-53 * cantidad);
    stepper1.run();}

else if (valuex >= -49 && valuex <= -45 )
 
 {stepper1.moveTo(-47 * cantidad);
    stepper1.run();}
       
else if (valuex >= -44 && valuex <= -40 )
 
 {stepper1.moveTo(-43 * cantidad);
    stepper1.run();}   
    
else if (valuex >= -39 && valuex <= -35 )
 
 {stepper1.moveTo(-37 * cantidad);
    stepper1.run();}    
    
else if (valuex >= -34 && valuex <= -30 )
 
 {stepper1.moveTo(-33 * cantidad);
    stepper1.run();}  

else if (valuex >= -29 && valuex <= -25 )
 
 {stepper1.moveTo(-27 * cantidad);
    stepper1.run();}   
    
else if (valuex >= -24 && valuex <= -20 )
 
 {stepper1.moveTo(-23 * cantidad);
    stepper1.run();}  
    
else if (valuex >= -19 && valuex <= -15 )
 
 {stepper1.moveTo(-17 * cantidad);
    stepper1.run();}  
    
else if (valuex >= -14 && valuex <= -10 )
 
 {stepper1.moveTo(-13 * cantidad);
    stepper1.run();} 
   
else if (valuex >= -9 && valuex <= -5 )
 
 {stepper1.moveTo(-7 * cantidad);
    stepper1.run();}   
    
else if (valuex >= -4 && valuex <= -1 )
 
 {stepper1.moveTo(-3 * cantidad);
    stepper1.run();}  

    
else if (valuex == 0 )
 
 {stepper1.moveTo(0 * cantidad);
    stepper1.run();} 


else if (valuex >= 1 && valuex <= 5 )
 
 {stepper1.moveTo(3 * cantidad);
    stepper1.run();}

else if (valuex >= 6 && valuex <= 10 )
 
 {stepper1.moveTo(8 * cantidad);
    stepper1.run();}

else if (valuex >= 11 && valuex <= 15 )
 
 {stepper1.moveTo(13 * cantidad);
    stepper1.run();}
    
else if (valuex >= 16 && valuex <= 20 )
 
 {stepper1.moveTo(18 * cantidad);
    stepper1.run();}
    
else if (valuex >= 21 && valuex <= 25 )
 
 {stepper1.moveTo(23 * cantidad);
    stepper1.run();}
    
else if (valuex >= 26 && valuex <= 30 )
 
 {stepper1.moveTo(28 * cantidad);
    stepper1.run();}
    
else if (valuex >= 31 && valuex <= 35 )
 
 {stepper1.moveTo(33 * cantidad);
    stepper1.run();}
    
else if (valuex >= 36 && valuex <= 40 )
 
 {stepper1.moveTo(38 * cantidad);
    stepper1.run();}
    
else if (valuex >= 41 && valuex <= 45 )
 
 {stepper1.moveTo(43 * cantidad);
    stepper1.run();}
    
else if (valuex >= 46 && valuex <= 50 )
 
 {stepper1.moveTo(48 * cantidad);
    stepper1.run();}
      
else if (valuex >= 51 && valuex <= 55 )
 
 {stepper1.moveTo(53 * cantidad);
    stepper1.run();}
      
else if (valuex >= 56 && valuex <= 60 )
 
 {stepper1.moveTo(58 * cantidad);
    stepper1.run();}

else if (valuex >= 61 && valuex <= 65 )
 
 {stepper1.moveTo(63 * cantidad);
    stepper1.run();}

else if (valuex >= 66 && valuex <= 70 )
 
 {stepper1.moveTo(68 * cantidad);
    stepper1.run();}

else if (valuex >= 71 && valuex <= 85 )
 
 {stepper1.moveTo(73 * cantidad);
    stepper1.run();}



else if (valuex >= 86 && valuex <= 89 )
 
 {stepper1.moveTo(88 * cantidad);
    stepper1.run();}

    
    
    
    
    

    

else {}

//////_---


}


else


{

if (digitalRead(LEFT_PIN) == 0) {
   
    stepper2.moveTo(pos);
    stepper2.run();  
    }
   
   else if (digitalRead(RIGHT_PIN) == 0) {    
   
    stepper2.moveTo(-pos);
    stepper2.run(); 
    }
   
   else if (digitalRead(STOP_PIN) == 0) {
   
    stepper2.moveTo(0);
    stepper2.run(); 
    }



}


  
  
  
}
