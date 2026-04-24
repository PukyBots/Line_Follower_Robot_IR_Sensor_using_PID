#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;         // Keep 1 for black line; change to 0 for white line
unsigned int numSensors = 5;  // Number of sensors (5-sensor array in this case)
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 215;
int currentSpeed = 150;
int sensorWeight[5] = {2, 1, 0, -1, -2};  // Weights for 5-sensor array
int activeSensors;
float Kp = 0.4;   // start here and tune       
float Kd = 8.0;   // start small then increase
float Ki = 0.0;

// float Kp = 0.3;   // start here and tune
// float Kd = 4.5;   // start small then increase
// float Ki = 0.0;

int onLine = 0;
int minValues[5], maxValues[5], threshold[5], sensorValue[5], sensorArray[5];

bool leftDetected, rightDetected;
unsigned long turnDetectTime;
unsigned int turnDetectInterval = 100;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  // Pushbutton
  pinMode(12, INPUT_PULLUP);  // Pushbutton
  pinMode(13, OUTPUT);        // LED

  pinMode(5, OUTPUT);     // Standby for older carrier boards
  digitalWrite(5, HIGH);  // Enables the motor driver
  
  // Set up sensor array for 5 sensors
  if (numSensors == 5) {
    sensorWeight[0] = 2;
    sensorWeight[1] = 1;
    sensorWeight[2] = 0;
    sensorWeight[3] = -1;
    sensorWeight[4] = -2;
  }
}

void loop() {
  while (digitalRead(11)) {}  // Wait for pushbutton press
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}  // Wait for pushbutton press
  delay(500);

  while (1) {
    readLine();
    if (numSensors == 5) {
      if (sensorArray[0] || sensorArray[4]) {
        if (sensorArray[0]) leftDetected = 1;
        if (sensorArray[4]) rightDetected = 1;
        turnDetectTime = millis();
      }
    }
    
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  // PID line-following logic
      linefollow();
      digitalWrite(13, HIGH);

      if (sensorArray[2] && millis() - turnDetectTime > turnDetectInterval) {
        leftDetected = 0;
        rightDetected = 0;
      }

    } else {
      digitalWrite(13, LOW);
      if (leftDetected) {
        lsp = -200;
        rsp = 200;
      } else if (rightDetected) {
        lsp = 200;
        rsp = -200;
      }
    }
    motor1run(lsp);
    motor2run(rsp);
    Serial.print(lsp);
    Serial.print("  ");
    Serial.println(rsp);
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 5) {
    for (int i = 0; i < 5; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  // Ensure speed stays within motor limits



  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
}

void calibrate() {
  for (int i = 0; i < 5; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000; i++) {
    motor1run(100);
    motor2run(-100);

    for (int i = 0; i < 5; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  if (numSensors == 5) {
    for (int i = 0; i < 5; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;  // Active if sensor value is greater than 500
      if (sensorArray[i]) onLine = 1;         // At least one sensor is on the line
    }
  }
}

//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
