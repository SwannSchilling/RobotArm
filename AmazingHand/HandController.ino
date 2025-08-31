#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  204   // ~1ms pulse (0°) 
#define SERVOMAX  409   // ~2ms pulse (180°)
#define SERVO_FREQ 50   // 50Hz

// Number of servos
#define NUM_SERVOS 8

void setup() {
  Serial.begin(115200);
  Serial.println("Robotic Hand Controller Starting...");
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize all servos to neutral position (90 degrees)
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 90);
  }
  
  delay(500);
  Serial.println("Ready for commands");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
}

void processCommand(String command) {
  if (command.startsWith("SERVO")) {
    // Format: "SERVO,ID,ANGLE"
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      int servoId = command.substring(firstComma + 1, secondComma).toInt();
      int angle = command.substring(secondComma + 1).toInt();
      
      if (servoId >= 0 && servoId < NUM_SERVOS && angle >= 0 && angle <= 180) {
        setServoAngle(servoId, angle);
        Serial.println("OK");
      } else {
        Serial.println("ERROR: Invalid servo ID or angle");
      }
    }
  }
  else if (command.startsWith("MULTI")) {
    // Format: "MULTI,ID1,ANGLE1,ID2,ANGLE2,..."
    processMultiServoCommand(command);
  }
  else {
    Serial.println("ERROR: Unknown command");
  }
}

void processMultiServoCommand(String command) {
  // Remove "MULTI," prefix
  String data = command.substring(6);
  
  int index = 0;
  bool success = true;
  
  while (data.length() > 0 && success) {
    int commaPos = data.indexOf(',');
    if (commaPos == -1) break;
    
    int servoId = data.substring(0, commaPos).toInt();
    data = data.substring(commaPos + 1);
    
    commaPos = data.indexOf(',');
    int angle;
    if (commaPos == -1) {
      // Last pair
      angle = data.toInt();
      data = "";
    } else {
      angle = data.substring(0, commaPos).toInt();
      data = data.substring(commaPos + 1);
    }
    
    if (servoId >= 0 && servoId < NUM_SERVOS && angle >= 0 && angle <= 180) {
      setServoAngle(servoId, angle);
    } else {
      success = false;
      Serial.println("ERROR: Invalid servo ID or angle in multi command");
      break;
    }
  }
  
  if (success) {
    Serial.println("OK");
  }
}

void setServoAngle(int servoNum, int angle) {
  // Convert angle (0-180) to pulse length
  int pulseLen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseLen);
  
  // Optional: Add small delay to prevent overwhelming the PCA9685
  delayMicroseconds(100);
}

// Optional: Function to read current servo positions (if needed)
void printServoPositions() {
  Serial.println("Current servo positions not available with PCA9685");
  // Note: PCA9685 is output-only, can't read servo positions
}