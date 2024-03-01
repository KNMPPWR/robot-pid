#include <ArduinoBLE.h>

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

bool robotIsRunning = false;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if(!BLE.begin()) {
    while (1);
  }

  Serial.println("Discovering...");
}

void loop() {
  BLEDevice peripheral;

  do {
    BLE.scanForUuid(bleServiceUuid);
    peripheral = BLE.available();
  } while(!peripheral);

  if(peripheral) {
    BLE.stopScan();

    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral) {
  if(!peripheral.connect()) {
    return;
  }

  if(!peripheral.discoverAttributes()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleLineSensorsChar = peripheral.characteristic(bleLineSensorsCharUuid);
  if(!bleLineSensorsChar || !bleLineSensorsChar.canRead()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleErrorChar = peripheral.characteristic(bleErrorCharUuid);
  if(!bleErrorChar || !bleErrorChar.canRead()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic blePidChar = peripheral.characteristic(blePidCharUuid);
  if(!blePidChar || !blePidChar.canRead()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleRunningChar = peripheral.characteristic(bleRunningCharUuid);
  if(!bleRunningChar || !bleRunningChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleKpChar = peripheral.characteristic(bleKpCharUuid);
  if(!bleKpChar || !bleKpChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleKiChar = peripheral.characteristic(bleKiCharUuid);
  if(!bleKiChar || !bleKiChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleKdChar = peripheral.characteristic(bleKdCharUuid);
  if(!bleKdChar || !bleKdChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleWeakReactChar = peripheral.characteristic(bleWeakReactCharUuid);
  if(!bleWeakReactChar || !bleWeakReactChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleStrongReactChar = peripheral.characteristic(bleStrongReactCharUuid);
  if(!bleStrongReactChar || !bleStrongReactChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  BLECharacteristic bleMaxErrorSumChar = peripheral.characteristic(bleMaxErrorSumCharUuid);
  if(!bleMaxErrorSumChar || !bleMaxErrorSumChar.canWrite()) {
    peripheral.disconnect();
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected!");
  
  int16_t lines;
  float error, pid;
  while (peripheral.connected()) {
    if(robotIsRunning) {
      bleLineSensorsChar.readValue(&lines, bleLineSensorsChar.valueLength());
      bleErrorChar.readValue(&error, bleErrorChar.valueLength());
      blePidChar.readValue(&pid, blePidChar.valueLength());

      Serial.print(lines);
      Serial.print('\t');       
      Serial.print(error);
      Serial.print('\t');
      Serial.print(pid);
      Serial.println('\t');
    }

    if(Serial.available() > 0) {
      char commandByte = Serial.read();
      if(commandByte == 'r') {
        robotIsRunning = !robotIsRunning;
        int ret = bleRunningChar.writeValue(&robotIsRunning, sizeof(robotIsRunning));
        Serial.print("Robot is running: ");
        Serial.println(robotIsRunning);
      }
      else if(commandByte == 'p') {
        float p = Serial.parseFloat();
        int ret = bleKpChar.writeValue(&p, sizeof(p));
        Serial.print("New p value: ");
        Serial.println(p);
      }
      else if(commandByte == 'i') {
        float i = Serial.parseFloat();
        int ret = bleKiChar.writeValue(&i, sizeof(i));
        Serial.print("New i value: ");
        Serial.println(i);
      }
      else if(commandByte == 'd') {
        float d = Serial.parseFloat();
        int ret = bleKdChar.writeValue(&d, sizeof(d));
        Serial.print("New d value: ");
        Serial.println(d);
      }
      else if(commandByte == 'w') {
        float w = Serial.parseFloat();
        int ret = bleWeakReactChar.writeValue(&w, sizeof(w));
        Serial.print("New weak reaction: ");
        Serial.println(w);
      }
      else if(commandByte == 's') {
        float s = Serial.parseFloat();
        int ret = bleStrongReactChar.writeValue(&s, sizeof(s));
        Serial.print("New strong reaction: ");
        Serial.println(s);
      }
      else if(commandByte == 'm') {
        float m = Serial.parseFloat();
        int ret = bleMaxErrorSumChar.writeValue(&m, sizeof(m));
        Serial.print("New max error sum: ");
        Serial.println(m);
      }
    }
  }

  Serial.println("Disconnected!");
  digitalWrite(LED_BUILTIN, LOW);
  robotIsRunning = false;
}