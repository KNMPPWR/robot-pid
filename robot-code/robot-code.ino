#include <ArduinoBLE.h>

#define MOTOR1_IN1_PIN  D2
#define MOTOR1_IN2_PIN  D3
#define MOTOR1_EN_PIN   A0

#define MOTOR2_IN1_PIN  D4
#define MOTOR2_IN2_PIN  D5
#define MOTOR2_EN_PIN   A1

#define SENSOR1_PIN D6
#define SENSOR2_PIN D7
#define SENSOR3_PIN D8
#define SENSOR4_PIN D9

#define MIN_MOTOR_VALUE   75
#define MAX_MOTOR_VALUE   255
#define ZERO_RANGE_VALUE  10
#define BASE_MOTOR_VALUE  90

const char* bleServiceUuid = "8f63a896-1d72-45ae-97af-a98b661b5598";
const char* bleLineSensorsCharUuid = "622795b2-a2e5-42d4-8c4f-3b2a5ecbd949";
const char* bleErrorCharUuid = "6d9cfeef-e694-4fe6-a584-ec6e5ce0bcaf";
const char* blePidCharUuid = "c9722940-10b3-41ef-89c7-3bd602c8b2ad";
const char* bleRunningCharUuid = "76e179e7-3aa2-4365-8224-f9cf6ec6b5e8";
const char* bleKpCharUuid = "d73f03b6-e0b7-4d8e-b85c-2fff407e3047";
const char* bleKiCharUuid = "bed9cdda-da6d-4182-acee-c48193b5aa2d";
const char* bleKdCharUuid = "80ea2424-7b2b-4ca0-8c84-b0c703680394";
const char* bleWeakReactCharUuid = "a08358fe-78fd-4400-9c8a-9b2c50175dec";
const char* bleStrongReactCharUuid = "64976aa9-0432-4a59-9b30-3c3a3ac32584";
const char* bleMaxErrorSumCharUuid = "82505045-1e9d-496b-ad53-be162464c5fc";

BLEService bleService(bleServiceUuid);
BLEIntCharacteristic bleLineSensorsChar(bleLineSensorsCharUuid, BLERead | BLENotify | BLEWrite);
BLEFloatCharacteristic bleErrorChar(bleErrorCharUuid, BLERead | BLENotify | BLEWrite);
BLEFloatCharacteristic blePidChar(blePidCharUuid, BLERead | BLENotify | BLEWrite);
BLEBooleanCharacteristic bleRunningChar(bleRunningCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleKpChar(bleKpCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleKiChar(bleKiCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleKdChar(bleKdCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleWeakReactChar(bleWeakReactCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleStrongReactChar(bleStrongReactCharUuid, BLERead | BLEWrite);
BLEFloatCharacteristic bleMaxErrorSumChar(bleMaxErrorSumCharUuid, BLERead | BLEWrite);

bool isRunning = false;

PinStatus sensor1State, sensor2State, sensor3State, sensor4State;

float kp = 1.0;
float ki = 1.0;
float kd = 1.0;

float lastError = 0;
float errorSum = 0;
float maxErrorSum = 100;

unsigned long lastTime = 0;

float weakReaction = 1;
float strongReaction = 4;

void turnOffMotors() {
  // left motor
  digitalWrite(MOTOR1_IN1_PIN, LOW);
  digitalWrite(MOTOR1_IN2_PIN, LOW);

  // right motor
  digitalWrite(MOTOR2_IN1_PIN, LOW);
  digitalWrite(MOTOR2_IN2_PIN, LOW);
}

void setMotors(int16_t leftMotorValue, int16_t rightMotorValue) {
  // left motor
  if(leftMotorValue < 0) {
    int16_t absValue = -leftMotorValue;
    if(absValue > MAX_MOTOR_VALUE) {
      absValue = MAX_MOTOR_VALUE;
    }
    else if(absValue < MIN_MOTOR_VALUE && absValue > ZERO_RANGE_VALUE) {
      absValue = MIN_MOTOR_VALUE;
    }
    else if(absValue < ZERO_RANGE_VALUE) {
      absValue = 0;
    }

    analogWrite(MOTOR1_EN_PIN, absValue);
    digitalWrite(MOTOR1_IN1_PIN, HIGH);
    digitalWrite(MOTOR1_IN2_PIN, LOW);
  }
  else {
    if(leftMotorValue > MAX_MOTOR_VALUE) {
      leftMotorValue = MAX_MOTOR_VALUE;
    }
    else if(leftMotorValue < MIN_MOTOR_VALUE && leftMotorValue > ZERO_RANGE_VALUE) {
      leftMotorValue = MIN_MOTOR_VALUE;
    }
    else if(leftMotorValue < ZERO_RANGE_VALUE) {
      leftMotorValue = 0;
    }

    analogWrite(MOTOR1_EN_PIN, leftMotorValue);
    digitalWrite(MOTOR1_IN1_PIN, LOW);
    digitalWrite(MOTOR1_IN2_PIN, HIGH);
  }

  // right motor
  if(rightMotorValue < 0) {
    int16_t absValue = -leftMotorValue;
    if(absValue > MAX_MOTOR_VALUE) {
      absValue = MAX_MOTOR_VALUE;
    }
    else if(absValue < MIN_MOTOR_VALUE && absValue > ZERO_RANGE_VALUE) {
      absValue = MIN_MOTOR_VALUE;
    }
    else if(absValue < ZERO_RANGE_VALUE) {
      absValue = 0;
    }

    analogWrite(MOTOR2_EN_PIN, absValue);
    digitalWrite(MOTOR2_IN1_PIN, HIGH);
    digitalWrite(MOTOR2_IN2_PIN, LOW);
  }
  else {
    if(rightMotorValue > MAX_MOTOR_VALUE) {
      rightMotorValue = MAX_MOTOR_VALUE;
    }
    else if(rightMotorValue < MIN_MOTOR_VALUE && rightMotorValue > ZERO_RANGE_VALUE) {
      rightMotorValue = MIN_MOTOR_VALUE;
    }
    else if(rightMotorValue < ZERO_RANGE_VALUE) {
      rightMotorValue = 0;
    }

    analogWrite(MOTOR2_EN_PIN, rightMotorValue);
    digitalWrite(MOTOR2_IN2_PIN, LOW);
    digitalWrite(MOTOR2_IN2_PIN, HIGH);
  }
}

void readSensors(PinStatus* value1, PinStatus* value2, PinStatus* value3, PinStatus* value4) {
  *value1 = digitalRead(SENSOR1_PIN);
  *value2 = digitalRead(SENSOR2_PIN);
  *value3 = digitalRead(SENSOR3_PIN);
  *value4 = digitalRead(SENSOR4_PIN);
}

float calulatePid(float error, float dt) {
  errorSum += dt * error;
  if(errorSum > maxErrorSum) {
    errorSum = maxErrorSum;
  }
  else if(errorSum < -maxErrorSum) {
    errorSum = -maxErrorSum;
  }

  float p = kp * error;
  float i = ki * errorSum;
  float d = kd * (error - lastError) / dt;

  lastError = error;

  return p + i + d;
}

void setup() {
  // Serial.begin(9600); 
  // while (!Serial);

  // builtin led initialization
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // left motor initialization
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR1_EN_PIN, OUTPUT);

  // right motor initialization
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_EN_PIN, OUTPUT);

  // line sensors initialization
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSOR3_PIN, INPUT);
  pinMode(SENSOR4_PIN, INPUT);

  // bluetooth connection
  if(!BLE.begin()) {
    while(1);
  }

  BLE.setLocalName("Pid - Robot");
  BLE.setAdvertisedService(bleService);

  bleService.addCharacteristic(bleLineSensorsChar);
  bleService.addCharacteristic(bleErrorChar);
  bleService.addCharacteristic(blePidChar);
  bleService.addCharacteristic(bleRunningChar);
  bleService.addCharacteristic(bleKpChar);
  bleService.addCharacteristic(bleKiChar);
  bleService.addCharacteristic(bleKdChar);
  bleService.addCharacteristic(bleWeakReactChar);
  bleService.addCharacteristic(bleStrongReactChar);
  bleService.addCharacteristic(bleMaxErrorSumChar);

  BLE.addService(bleService);

  bleRunningChar.setValue(isRunning);
  bleKpChar.setValue(kp);
  bleKiChar.setValue(ki);
  bleKdChar.setValue(kd);
  bleWeakReactChar.setValue(weakReaction);
  bleStrongReactChar.setValue(strongReaction);
  bleMaxErrorSumChar.setValue(maxErrorSum);

  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);

    while (central && central.connected()) {
      if(bleRunningChar.written()) {
        isRunning = bleRunningChar.value();
      }
      if(bleKpChar.written()) {
        kp = bleKpChar.value();
      }
      if(bleKiChar.written()) {
        ki = bleKiChar.value();
      }
      if(bleKdChar.written()) {
        kd = bleKdChar.value();
      }
      if(bleWeakReactChar.written()) {
        weakReaction = bleWeakReactChar.value();
      }
      if(bleStrongReactChar.written()) {
        strongReaction = bleStrongReactChar.value();
      }
      if(bleMaxErrorSumChar.written()) {
        maxErrorSum = bleMaxErrorSumChar.value();
      }

      if(isRunning) {
        readSensors(&sensor1State, &sensor2State, &sensor3State, &sensor4State);

        float currentError = 0;
        int16_t lines = 0;
        if(sensor1State == HIGH) { 
          currentError -= strongReaction;
          lines |= 8;
        }
        if(sensor2State == HIGH) { 
          currentError -= weakReaction;
          lines |= 4;
        }
        if(sensor3State == HIGH) { 
          currentError += weakReaction;
          lines |= 2;
        }
        if(sensor4State == HIGH) { 
          currentError += strongReaction;
          lines |= 1;
        }

        if(lastTime == 0) {
          lastTime = millis();
        }

        unsigned long currentTime = millis();
        float dt = (float)lastTime - currentTime;
        float pid = calulatePid(currentError, dt);
        lastTime = currentTime;

        int16_t diff = pid / 2;

        setMotors(BASE_MOTOR_VALUE + diff, BASE_MOTOR_VALUE - diff);

        bleLineSensorsChar.writeValue(lines);
        bleErrorChar.writeValue(currentError);
        blePidChar.writeValue(pid);

        delay(100); 
      }  
      else {
        turnOffMotors();
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
  }
}