#include <AccelStepper.h>


/////kalman---------------------------------------------
#include <Wire.h>
#include "Kalman.h" 
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - 
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
//////Kalman-------------------------------------------------


AccelStepper stepper1(1, 9, 8);

AccelStepper stepper2(1, 7, 6);
int pos = 2000;
int cantidad = 60;

// Define our three input button pins
#define  LEFT_PIN  4
#define  STOP_PIN  3
#define  RIGHT_PIN 2

#define  SPEED_PIN 0

// Define our maximum and minimum speed in steps per second (scale pot to these)
#define  MAX_SPEED 1500
#define  MIN_SPEED 0.1


void setup() {

  // The only AccelStepper value we have to set here is the max speeed, which is higher than we'll ever go
  stepper1.setMaxSpeed(10000.0);
  stepper2.setMaxSpeed(10000.0);
  
    stepper1.setAcceleration(1000);
      stepper2.setAcceleration(1000);
 
  // Set up the three button inputs, with pullups
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  
////////kalmen---------------------------------------------------------------------
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

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

  timer = micros();
  //////////////////kalman-----------------------------------------
}

void loop() {
  
  
  static char sign = 0;                     


////////////kalman--------------------------------------------------

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

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
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
#if 1// Set to 1 to activate
  //Serial.print(accX); Serial.print("\t");
  //Serial.print(accY); Serial.print("\t");
  //Serial.print(accZ); Serial.print("\t");

  //Serial.print(gyroX); Serial.print("\t");
  //Serial.print(gyroY); Serial.print("\t");
  //Serial.print(gyroZ); Serial.print("\t");

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
  
  ////////////////////-kalman-----------------------------------------


 if (digitalRead(LEFT_PIN) == 0) {
    sign = 1;
    stepper2.moveTo(pos);
    stepper2.run();  
    }
   
   else if (digitalRead(RIGHT_PIN) == 0) {    
    sign = -1;
    stepper2.moveTo(-pos);
    stepper2.run(); 
    }
   
   else if (digitalRead(STOP_PIN) == 0) {
    sign = 0;
    stepper2.moveTo(0);
    stepper2.run(); 
    }



else {



/////-----
int valuey = kalAngleY;  
 
 if (valuey > -89 && valuey < -80 )
 
 {stepper2.moveTo(-85 * cantidad);
    stepper2.run();}


   else if (valuey > -79 && valuey < -70 )
 
 {stepper2.moveTo(-75 * cantidad);
    stepper2.run();}

   else if (valuey > -69 && valuey < -60 )
 
 {stepper2.moveTo(-65 * cantidad);
    stepper2.run();}

   else if (valuey > -59 && valuey < -50 )
 
 {stepper2.moveTo(-55 * cantidad);
    stepper2.run();}

   else if (valuey > -49 && valuey < -40 )
 
 {stepper2.moveTo(-45 * cantidad);
    stepper2.run();}

   else if (valuey > -39 && valuey < -30 )
 
 {stepper2.moveTo(-35 * cantidad);
    stepper2.run();}

   else if (valuey > -29 && valuey < -20 )
 
 {stepper2.moveTo(-25 * cantidad);
    stepper2.run();}

   else if (valuey > -19 && valuey < -10 )
 
 {stepper2.moveTo(-15 * cantidad);
    stepper2.run();}

   else if (valuey > -9 && valuey < 0 )
 
 {stepper2.moveTo(-5 * cantidad);
    stepper2.run();}

   else if (valuey > 1 && valuey < 10 )
 
 {stepper2.moveTo(5 * cantidad);
    stepper2.run();}

   else if (valuey > 11 && valuey < 20 )
 
 {stepper2.moveTo(15 * cantidad);
    stepper2.run();}

   else if (valuey > 21 && valuey < 30 )
 
 {stepper2.moveTo(25 * cantidad);
    stepper2.run();}

      
 else if (valuey > 31 && valuey < 40 )
 
 {stepper2.moveTo(35 * cantidad);
    stepper2.run();}
      
      
 else if (valuey > 41 && valuey < 50 )
 
 {stepper2.moveTo(45 * cantidad);
    stepper2.run();}

      
 else if (valuey > 51 && valuey < 60 )
 
 {stepper2.moveTo(55 * cantidad);
    stepper2.run();}

 else if (valuey > 61 && valuey < 70 )
 
 {stepper2.moveTo(65 * cantidad);
    stepper2.run();}

 else if (valuey > 71 && valuey < 80 )
 
 {stepper2.moveTo(75 * cantidad);
    stepper2.run();}

 else if (valuey > 81 && valuey < 89 )
 
 {stepper2.moveTo(85 * cantidad);
    stepper2.run();}

////

int valuex = kalAngleX;  
 
 if (valuex > -89 && valuex < -80 )
 
 {stepper2.moveTo(-85 * cantidad);
    stepper2.run();}


   else if (valuex > -79 && valuex < -70 )
 
 {stepper2.moveTo(-75 * cantidad);
    stepper2.run();}

   else if (valuex > -69 && valuex < -60 )
 
 {stepper2.moveTo(-65 * cantidad);
    stepper2.run();}

   else if (valuex > -59 && valuex < -50 )
 
 {stepper2.moveTo(-55 * cantidad);
    stepper2.run();}

   else if (valuex > -49 && valuex < -40 )
 
 {stepper2.moveTo(-45 * cantidad);
    stepper2.run();}

   else if (valuex > -39 && valuex < -30 )
 
 {stepper2.moveTo(-35 * cantidad);
    stepper2.run();}

   else if (valuex > -29 && valuex < -20 )
 
 {stepper2.moveTo(-25 * cantidad);
    stepper2.run();}

   else if (valuex > -11 && valuex < -10 )
 
 {stepper2.moveTo(-15 * cantidad);
    stepper2.run();}

   else if (valuex > -9 && valuex < 0 )
 
 {stepper2.moveTo(-5 * cantidad);
    stepper2.run();}

   else if (valuex > 1 && valuex < 10 )
 
 {stepper2.moveTo(5 * cantidad);
    stepper2.run();}

   else if (valuex > 11 && valuex < 20 )
 
 {stepper2.moveTo(15 * cantidad);
    stepper2.run();}

   else if (valuex > 21 && valuex < 30 )
 
 {stepper2.moveTo(25 * cantidad);
    stepper2.run();}

      
 else if (valuex > 31 && valuex < 40 )
 
 {stepper2.moveTo(35 * cantidad);
    stepper2.run();}
      
      
 else if (valuex > 41 && valuex < 50 )
 
 {stepper2.moveTo(45 * cantidad);
    stepper2.run();}

      
 else if (valuex > 51 && valuex < 60 )
 
 {stepper2.moveTo(55 * cantidad);
    stepper2.run();}

 else if (valuex > 61 && valuex < 70 )
 
 {stepper2.moveTo(65 * cantidad);
    stepper2.run();}

 else if (valuex > 71 && valuex < 80 )
 
 {stepper2.moveTo(75 * cantidad);
    stepper2.run();}

 else if (valuex > 81 && valuex < 89 )
 
 {stepper2.moveTo(85 * cantidad);
    stepper2.run();}


//////_---



}
 

 

 
}
