Materials

2 x L298N Driver  
2 x Stepper motor 4 wire  
1 X Arduino R3  
1 x Gyroscope/Accelerometer ITG GY-521 MPU6050

-------------------------
Software

I2C.ino and Kalman.h MUST be in the same directory that sketch

-------------------------

Pinout

ITG Gy-521 to Arduino 

ITG pin SCL ---> Pin5 Arduino  
ITG pin SDA ---> Pin4 Arduino      
ITG VCC ----> 5v Arduino    
ITG GND ---> GND Arduino 


1st L298N to Arduino

L298N pin IN1 ---> Pin 11 Arduino  
L298N pin IN2 ---> Pin 10 Arduino  
L298N pin IN3 ---> Pin  9 Arduino  
L298N pin IN4 ---> Pin  8 Arduino   

L298N Pin GND ---> Pin GND Arduino (please don't forget this connection, otherwise the motor it doesn't work)

2nd L298N to Arduino

L298N pin IN1 ---> Pin 4 Arduino  
L298N pin IN2 ---> Pin 5 Arduino  
L298N pin IN3 ---> Pin 6 Arduino  
L298N pin IN4 ---> Pin 7 Arduino 

L298N to Motor

First you will need figure out what are the coils, take a multimeter and change to ohmmeter, 
then determine which pairs are group A and group B 

then

 L298N Out1/out2 ---> to Group A motor  
 L298N Out3/Out4 ---> to Group B motor  

-------------------------------------------
 
 LN982N and Power supply

You will need to connect L298N to external power source 5vdc or 12Vdc , it depends of motor voltage.
