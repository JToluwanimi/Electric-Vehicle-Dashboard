#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define Serial Communication for GPS and DWIN
SoftwareSerial gpsSerial(8, 9); // RX, TX for GPS
SoftwareSerial dwinSerial(2, 3); // RX, TX for DWIN Display

// Initialize GPS Module
TinyGPSPlus gps;

// LED Pin
const int ledPin = 5;

// Battery Voltage Measurement
const int batteryPin = A0;
const float maxVoltage = 56.0;  // Full battery voltage
const float minVoltage = 46.0;  // Low battery voltage

// Voltage divider resistors
const float R1 = 100000.0; // 100k ohms
const float R2 = 10000.0;  // 10k ohms

// GPS Data Variables
double prevLat = 0.0, prevLon = 0.0;
double totalDistance = 0.0;
bool firstFix = true;
unsigned long prevTime = 0;

// Speed Filter
double previousSpeed = 0.0;
const float alpha = 0.2; // Low-pass filter constant

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);       // Debugging (Optional)
  gpsSerial.begin(9600);      // GPS Module
  dwinSerial.begin(115200);   // DWIN HMI
}

void loop() {
  readGPSData();
  delay(50);
  //float batteryPercentage = ;
  sendBatteryToDWIN(readBatteryPercentage());
  delay(50);
  checkLEDCommand();
  delay(50);
}

// -------- GPS Reading & Processing --------
void readGPSData() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      double currentLat = gps.location.lat();
      double currentLon = gps.location.lng();
      
      // Calculate Distance
      if (firstFix) {
        prevLat = currentLat;
        prevLon = currentLon;
        firstFix = false;
      } else {
        double distance = TinyGPSPlus::distanceBetween(prevLat, prevLon, currentLat, currentLon) / 1000.0;
        totalDistance += distance;
        prevLat = currentLat;
        prevLon = currentLon;
      }

      // Calculate Speed
      unsigned long currentTime = millis();
      if (currentTime - prevTime >= 1000) { // Update every second
        double rawSpeed = gps.speed.kmph();
        int filteredSpeed = (alpha * rawSpeed) + ((1 - alpha) * previousSpeed);
        previousSpeed = filteredSpeed;
        prevTime = currentTime;

        sendSpeedToDWIN(filteredSpeed);
        sendDistanceToDWIN(totalDistance);
      }
    }
  }
}

// -------- Battery Voltage Processing --------
int readBatteryPercentage() {
  int rawValue = analogRead(batteryPin);
  float voltage = rawValue * (5.0 / 1023.0) * ((R1 + R2) / R2); // Scaling based on voltage divider
  int percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  return constrain(percentage, 0, 100);
}

// -------- DWIN Command Processing --------
const int bufferSize = 9;

void checkLEDCommand() {
  if (dwinSerial.available() >= bufferSize) {
    byte command[9];
    dwinSerial.readBytes(command, 9);
    if (command[0] == 0x5A && command[1] == 0xA5) { // DWIN Protocol Check
      // if (command[2] == 0x04 && command[3] == 0x83) { // LED Control Address
       switch (command[4])
      {
        case 0x51:   // Command for built-in LED
          if (command[8] == 1)
          {
            digitalWrite(ledPin, HIGH);   // Turn the built-in LED ON
            Serial.println("Built-in LED ON"); // Output status to the serial monitor
          }
          else
          {
            digitalWrite(ledPin, LOW);    // Turn the built-in LED OFF
            Serial.println("Built-in LED OFF"); // Output status to the serial monitor
          }
          break;
        
        default:
          Serial.println("Unrecognized command"); // Output a message for unrecognized commands
          break;
      }
      // }
    }
  }
  delay(10);  // Brief delay to allow for serial data processing
}


// -------- Sending Data to DWIN --------
void sendSpeedToDWIN(int speed) {
  int speedInt = static_cast<int>(speed);
  byte command[] = {0x5A, 0xA5, 0x07, 0x82, 0x53, 0x00, (byte)(speedInt >> 8), (byte)(speedInt & 0xFF)};
  dwinSerial.write(command, sizeof(command));
}

void sendDistanceToDWIN(int distance) {
  int distanceInt = static_cast<int>(distance);
  byte command[] = {0x5A, 0xA5, 0x07, 0x82, 0x54, 0x00, (byte)(distanceInt >> 8), (byte)(distanceInt & 0xFF)};
  dwinSerial.write(command, sizeof(command));
}

void sendBatteryToDWIN(int percentage) {
  int batteryInt = static_cast<int>(percentage);
  byte command[] = {0x5A, 0xA5, 0x07, 0x82, 0x52, 0x00, (byte)(batteryInt >> 8), (byte)(batteryInt & 0xFF)};
  dwinSerial.write(command, sizeof(command));
}
