#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//
const int motor1pin1 = 13;
const int motor1pin2 = 3;
const int enable1 = 9;

//
const int motor2pin1 = 4;
const int motor2pin2 = 5;
const int enable2 = 10;

//  Interrupt pin for button
const int buttonPin = 2;

//  Temperature sensor
const int tempPin = A1;

//  Ultrasonic front sensor
const int trigPinF = 11;
const int echoPinF = 12;

//  Ultrasonic mid sensor
const int trigPinM = A4;
const int echoPinM = A3;

//  Ultrasonic back sensor
//  const int trigPinB = 6;
//  const int echoPinB = 7;

//  Distance to stop the car
const float OBSTACLE_THRESHOLD = 20.0;

//  Parking spot detection parameters
const float LATERAL_DEPTH_THRESHOLD = 40.0;  // Minimum depth to detect a parking spot
const float LATERAL_PARK_LENGTH = 40.0;     // Minimum length for lateral parking (cm)
const float REVERSE_PARK_LENGTH = 20.0;     // Minimum length for reverse parking (cm)
const float CAR_SPEED_CM_PER_MS = 0.05;      // Approximate car speed for distance calculation

//  Button state: 0 = idle, 1 = forward, 2 = found lateral park, 3 = found reverse park
volatile int state = 0;

//  Stored temperature
float tempC = 0;

//  Parking spot detection variables
bool inParkingSpot = false;
unsigned long spotStartTime = 0;
float spotLength = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(enable1, OUTPUT);

  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enable2, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), changeState, FALLING);

  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  
  pinMode(trigPinM, OUTPUT);
  pinMode(echoPinM, INPUT);
  
  //  Set up default speed for motors
  analogWrite(enable1, 200);
  analogWrite(enable2, 200);
  
  //   Set up LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  //  Make sure motors are off initially
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  
  //   Read the temperature
  int raw = analogRead(tempPin);
  raw = analogRead(tempPin);
  tempC = raw * (5.0 / 1024.0) * 100.0 - 15.0;
}

void loop() {
  //  Idle state
  if (state == 0) {
    //  Ensure motors are stopped in idle state
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
    
    //  Print temperature in idle state
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempC, 1);
    lcd.print(" C    ");
    lcd.setCursor(0, 1);
    lcd.print("Press button    ");
    delay(200);
  
  //   Parking search initiated
  } else if (state == 1) {    
    //  Read distances
    float frontDistance = readFrontDistance();
    float midDistance = readMidDistance();
    
    //  Check for obstacle in front
    if (frontDistance <= OBSTACLE_THRESHOLD) {
      //  Stop motors when obstacle detected within 20 cm
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, LOW);

      //  Return to idle
      state = 0;
      
      //  Alert user that there is no parking space
      lcd.setCursor(0, 0);
      lcd.print("Couldn't find   ");
      lcd.setCursor(0, 1);
      lcd.print("parking spot!   ");
  
      //  Additional delay to display message
      delay(2000);
    } else {
      //  Check for parking spot using middle sensor
      checkParkingSpot(midDistance);
    }
  }
  //  Found lateral parking spot
  else if (state == 2) {
    //  Stop motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
    
    //  Display lateral parking message
    lcd.setCursor(0, 0);
    lcd.print("Lateral park    ");
    lcd.setCursor(0, 1);
    lcd.print("found! L:");
    lcd.print((int)spotLength);
    lcd.print("cm   ");
    
    //  Additional delay to display message
    delay(2000);
  }
  //  Found reverse parking spot
  else if (state == 3) {
    //  Stop motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
    
    //  Display reverse parking message
    lcd.setCursor(0, 0);
    lcd.print("Reverse park    ");
    lcd.setCursor(0, 1);
    lcd.print("found! L:");
    lcd.print((int)spotLength);
    lcd.print("cm   ");
    //  Additional delay to display message
    delay(2000);
  }
}

void checkParkingSpot(float midDistance) {
  if (!inParkingSpot && midDistance >= LATERAL_DEPTH_THRESHOLD) {
    //  Started detecting a parking spot
    inParkingSpot = true;
    spotStartTime = millis();
  } 
  else if (inParkingSpot && midDistance < LATERAL_DEPTH_THRESHOLD) {
    //  End of parking spot detected
    inParkingSpot = false;
    
    //  Calculate spot length based on time spent measuring
    unsigned long spotDuration = millis() - spotStartTime;
    spotLength = spotDuration * CAR_SPEED_CM_PER_MS;
    
    //  Determine parking type based on length
    if (spotLength >= LATERAL_PARK_LENGTH) {
      state = 2; // Lateral park
    } else if (spotLength >= REVERSE_PARK_LENGTH) {
      state = 3; // Reverse park
    }
    //  If too small, continue searching (don't change state)
  }
}

void changeState() {
  static unsigned long lastInterrupt = 0;
  unsigned long currentTime = millis();
  
  //  Debounce button
  if (currentTime - lastInterrupt > 200) {
    //  Switch from idle to park search mode
    if (state == 0) {
      //  Read temperature
      int raw = analogRead(tempPin);
      raw = analogRead(tempPin);
      tempC = raw * (5.0 / 1024.0) * 100.0 - 15.0;

      //  Reset parking detection variables
      inParkingSpot = false;
      spotLength = 0;

      //  Change state
      state = 1;
      
      //  Start motors
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, HIGH);

      //   Set speed
      analogWrite(enable1, 200);
      analogWrite(enable2, 200);
  
    //  Switch from any active mode back to idle
    } else {
      //  Change state
      state = 0;
      
      //  Immediately stop motors
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, LOW);
      
      //  Reset parking detection variables
      inParkingSpot = false;
      spotLength = 0;
    }

    lastInterrupt = currentTime;
  }
}

float readFrontDistance() {
  digitalWrite(trigPinF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinF, LOW);
  
  //  Read distance with a 30 ms timeout
  float duration = pulseIn(echoPinF, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return (duration * 0.0343) / 2;
}

float readMidDistance() {
  digitalWrite(trigPinM, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinM, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinM, LOW);
  
  //  Read distance with a 30 ms timeout
  float duration = pulseIn(echoPinM, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return (duration * 0.0343) / 2;
}