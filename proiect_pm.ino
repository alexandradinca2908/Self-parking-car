#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  Left motors
const int motor1pin1 = 13;
const int motor1pin2 = 3;
const int enable1 = 9;

//  Right motors
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
const int trigPinM = A3;
const int echoPinM = A2;

//  Ultrasonic back sensor
const int trigPinB = 6;
const int echoPinB = 7;

//  Distance to stop the car
const float OBSTACLE_THRESHOLD = 20.0;

//  Parking spot detection parameters
const float LATERAL_DEPTH_THRESHOLD = 20.0;  // Minimum depth to detect a parking spot
const float LATERAL_PARK_LENGTH = 40.0;      // Minimum length for lateral parking (cm)
const float REVERSE_PARK_LENGTH = 20.0;      // Minimum length for reverse parking (cm)
const float CAR_SPEED_CM_PER_MS = 0.04;      // Approximate car speed for distance calculation

//  Parking parameters
const float BACK_OBSTACLE_THRESHOLD = 15.0;  // Stop when 15cm from back obstacle
const int ROTATION_TIME_45_DEG = 1000;       // Time in ms for 45 degree rotation

//  Button state: 0 = idle, 1 = forward, 2 = found lateral park, 3 = found reverse park
volatile int state = 0;

//  Stored temperature
float tempC = 0;

//  Parking spot detection variables
bool scanningForPark = false;
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
  
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  
  //  Set up default speed for motors
  analogWrite(enable1, 200);
  analogWrite(enable2, 200);
  
  //   Set up LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  //  Make sure motors are off initially
  stopMotors();
  
  //   Read the temperature
  int raw = analogRead(tempPin);
  raw = analogRead(tempPin);
  tempC = raw * (5.0 / 1024.0) * 100.0 - 15.0;
}

void loop() {
  //  Idle state
  if (state == 0) {
    //  Ensure motors are stopped in idle state
    stopMotors();
    
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
    //  Clear LCD
    lcd.clear();

    //  Read distances
    float frontDistance = readFrontDistance();
    float midDistance = readMidDistance();
    
    //  Check for obstacle in front
    if (frontDistance <= OBSTACLE_THRESHOLD) {
      //  Stop motors when obstacle detected within 20 cm
      stopMotors();

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
    stopMotors();
    
    //  Display lateral parking message
    lcd.setCursor(0, 0);
    lcd.print("Lateral park    ");
    lcd.setCursor(0, 1);
    lcd.print("found!          ");
    
    //  Additional delay to display message
    delay(2000);
    
    //  Execute lateral parking maneuver
    lateralPark();
    
    //  Return to idle state after parking
    state = 0;
  }
  //  Found reverse parking spot
  else if (state == 3) {
    //  Stop motors
    stopMotors();
    
    //  Display reverse parking message
    lcd.setCursor(0, 0);
    lcd.print("Reverse park    ");
    lcd.setCursor(0, 1);
    lcd.print("found!          ");

    //  Additional delay to display message
    delay(2000);
    
    //  Return to idle state
    state = 0;
  }
}

void lateralPark() {
  //  Rotate 45 degrees counter-clockwise
  rotateCounterClockwise45();
  delay(500);
  
  backUpUntilObstacle();
  delay(500);
  
  //  Rotate 45 degrees clockwise
  rotateClockwise45();
  delay(500);
  
  //  Parking complete
  lcd.setCursor(0, 0);
  lcd.print("Parking         ");
  lcd.setCursor(0, 1);
  lcd.print("complete!       ");
  delay(2000);
}

void rotateClockwise45() {
  //  Set motor speeds for rotation
  analogWrite(enable1, 200);
  analogWrite(enable2, 200);
  
  //  Rotate clockwise
  //  Left motor forward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  // Right motor backward
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  
  //  Rotate for calculated time
  delay(ROTATION_TIME_45_DEG);
  
  //  Stop motors
  stopMotors();
}

void rotateCounterClockwise45() {
  //  Set motor speeds for rotation
  analogWrite(enable1, 200);
  analogWrite(enable2, 200);
  
  //  Rotate counter-clockwise
  //  Left motor backward
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  //  Right motor forward
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  
  //  Rotate for calculated time
  delay(ROTATION_TIME_45_DEG);
  
  //  Stop motors
  stopMotors();
}

void backUpUntilObstacle() {
  //  Set motor speeds for backing up
  analogWrite(enable1, 200);
  analogWrite(enable2, 200);
  
  //  Start backing up
  goBackward();
  
  //  Keep backing up until back sensor detects obstacle at 10cm
  while (true) {
    float backDistance = readBackDistance();
    
    //  Check if we've reached the target distance
    if (backDistance <= BACK_OBSTACLE_THRESHOLD) {
      break;
    }
    
    //  Small delay to avoid overwhelming the sensor
    delay(50);
  }
  
  //  Stop motors
  stopMotors();
}

void goForward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void goBackward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void checkParkingSpot(float midDistance) {
  if (!scanningForPark && midDistance >= LATERAL_DEPTH_THRESHOLD) {
    //  Started detecting a parking spot
    scanningForPark = true;
    spotStartTime = millis();
  }
  else if (scanningForPark && midDistance < LATERAL_DEPTH_THRESHOLD) {
    //  End of parking spot detected
    scanningForPark = false;
    
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
      //  Reset parking detection variables
      scanningForPark = false;
      spotLength = 0;

      //  Change state
      state = 1;
      
      //  Start motors
      goForward();

      //   Set speed
      analogWrite(enable1, 200);
      analogWrite(enable2, 200);
  
    //  Switch from any active mode back to idle
    } else {
      //  Change state
      state = 0;
      
      //  Immediately stop motors
      stopMotors();
      
      //  Reset parking detection variables
      scanningForPark = false;
      spotLength = 0;

      //  Read temperature
      int raw = analogRead(tempPin);
      raw = analogRead(tempPin);
      tempC = raw * (5.0 / 1024.0) * 100.0 - 15.0;
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

float readBackDistance() {
  digitalWrite(trigPinB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinB, LOW);
  
  //  Read distance with a 30 ms timeout
  float duration = pulseIn(echoPinB, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return (duration * 0.0343) / 2;
}