
void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(12, INPUT); 
  pinMode(2, OUTPUT);  // relay for +/- 24V on pin#2
}


void loop() {
  while (digitalRead(12)){  // check the switch or jumper on pin#12
  
  /* tick 1 minute backwards every 1 second */
  
    digitalWrite(13, HIGH);   // set the LED on
    digitalWrite(2, HIGH);   // tick
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // set the LED off
    digitalWrite(2, LOW);   // tick
    delay(1000);              // wait for a second
  }
  
  /* Something COOLER */
  
  float something = millis()/60000.0;
  int value = 128.0 + 128 * sin( something * 2.0 * PI  );
// analogWrite(Pin,value);

  digitalWrite(13, HIGH);  // set the LED on
  digitalWrite(2, HIGH);   // tick
  delay(250+(value*10));   // sine wave delay
  Serial.println(250+(value*10));
  digitalWrite(13, LOW);   // set the LED off
  digitalWrite(2, LOW);    // tick
  delay(250+(value*10));   // sine wave delay
    
}
