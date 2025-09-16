/*
 * RC Car - Gamepad Control - RECEIVER
 * This sketch receives control data from the transmitter and uses it to
 * control the drive motor (TB6612FNG) and steering servo.
 *
 * === WIRING GUIDE ===
 * 
 * TB6612FNG Motor Driver:
 * VCC    -> 3.3V (ESP32)
 * GND    -> GND (ESP32)
 * VM     -> Motor power supply (6-15V) - Connect to battery positive
 * GND    -> Battery negative (share with ESP32 GND)
 * AO1    -> Motor wire 1
 * AO2    -> Motor wire 2
 * AIN1   -> GPIO 25 (ESP32)
 * AIN2   -> GPIO 26 (ESP32) 
 * PWMA   -> GPIO 33 (ESP32)
 * STBY   -> GPIO 32 (ESP32)
 * 
 * NRF24L01 Radio:
 * VCC    -> 3.3V
 * GND    -> GND
 * CE     -> GPIO 4
 * CSN    -> GPIO 5
 * SCK    -> GPIO 18
 * MOSI   -> GPIO 23
 * MISO   -> GPIO 19
 * 
 * Steering Servo:
 * Red    -> 5V or external power
 * Brown  -> GND
 * Orange -> GPIO 27 (ESP32)
 * 
 * MPU6500 Gyroscope:
 * VCC    -> 3.3V (ESP32)
 * GND    -> Common ground
 * SCL    -> GPIO 22 (I2C Clock)
 * SDA    -> GPIO 21 (I2C Data)
 * 
 * IMPORTANT: Make sure all GND connections are shared!
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// A structure to hold the control data (must match transmitter)
struct ControlData {
  int steer;
  int throttle;
};

ControlData receivedData;

// Radio
RF24 radio(4, 5); // CE, CSN
const byte address[6] = "00001";

// Motor Driver Pins
const int STBY = 32;
const int PWMA = 33;
const int AIN1 = 25;
const int AIN2 = 26;

// Servo
Servo steeringServo;
const int servoPin = 27;

// MPU6500 Gyroscope
MPU6050 mpu6050(Wire);

// Gyro control variables
bool gyroStabilization = false; // TEMPORARILY DISABLED for testing - Enable/disable gyro stabilization
float targetAngle = 0; // Desired angle (0 = straight)
float gyroCorrection = 0; // Correction value from gyro

// Gyro test function
void testGyro() {
  Serial.println("🧭 Testing MPU6500 gyroscope...");
  
  Serial.println("📊 Reading gyro values for 10 seconds...");
  for(int i = 0; i < 100; i++) {
    mpu6050.update();
    
    Serial.print("Angle Z: "); Serial.print(mpu6050.getAngleZ());
    Serial.print("° | Gyro Z: "); Serial.print(mpu6050.getGyroZ());
    Serial.print(" | Accel X: "); Serial.print(mpu6050.getAccX());
    Serial.print(" | Accel Y: "); Serial.println(mpu6050.getAccY());
    
    delay(100);
  }
  
  Serial.println("🧭 Gyro test complete!");
  
  // Reset angle to 0
  mpu6050.calcGyroOffsets(true);
  Serial.println("✅ Gyro calibrated and zeroed");
}

// Servo test function to verify servo wiring and movement
void testServo() {
  Serial.println("🔧 Testing servo motor FULL RANGE...");
  
  // Center position (90 degrees)
  Serial.println("Moving to CENTER (90°)...");
  steeringServo.write(90);
  delay(1500);
  
  // Full left turn (0 degrees)
  Serial.println("Moving to FULL LEFT (0°)...");
  steeringServo.write(0);
  delay(1500);
  
  // Full right turn (180 degrees)
  Serial.println("Moving to FULL RIGHT (180°)...");
  steeringServo.write(180);
  delay(1500);
  
  // Back to center
  Serial.println("Back to CENTER (90°)...");
  steeringServo.write(90);
  delay(1500);
  
  // Test intermediate positions
  Serial.println("Testing intermediate positions...");
  for(int angle = 0; angle <= 180; angle += 30) {
    Serial.print("Position: "); Serial.print(angle); Serial.println("°");
    steeringServo.write(angle);
    delay(500);
  }
  
  // Return to center
  steeringServo.write(90);
  Serial.println("Servo FULL RANGE test complete!");
}

// Motor test function to verify TB6612FNG wiring
void testMotor() {
  Serial.println("🔧 Testing motor driver...");
  
  // Test forward
  Serial.println("Testing FORWARD...");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 100); // Low speed test
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  delay(1000);
  
  // Test reverse
  Serial.println("Testing REVERSE...");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 100); // Low speed test
  delay(2000);
  
  // Stop
  Serial.println("STOP - Motor test complete");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  Serial.println("=== RC Car Receiver - Communication Test ===");
  Serial.println("Testing compatibility with transmitter...");

  // Motor Pin Setup
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  // Initially disable motor driver for safety
  digitalWrite(STBY, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  
  Serial.println("✅ Motor pins configured");

  // Servo Setup with proper pulse width for standard servos
  Serial.println("⚙️ Initializing servo motor...");
  steeringServo.setPeriodHertz(50); // Standard 50Hz servo frequency
  steeringServo.attach(servoPin, 500, 2500); // Min: 500μs, Max: 2500μs pulse width
  
  // Test servo FULL range movement
  Serial.println("🔄 Testing servo FULL RANGE movement...");
  steeringServo.write(90); // Center position
  delay(1000);
  Serial.println("   Center (90°)");
  
  steeringServo.write(0); // Full left
  delay(1000);
  Serial.println("   Full Left (0°)");
  
  steeringServo.write(180); // Full right
  delay(1000);
  Serial.println("   Full Right (180°)");
  
  steeringServo.write(90); // Back to center
  delay(1000);
  Serial.println("   Back to Center (90°)");
  
  Serial.println("✅ Servo attached, tested FULL RANGE, and centered at 90°");

  // MPU6500 Setup
  Serial.println("🧭 Initializing MPU6500 gyroscope...");
  Wire.begin(21, 22); // SDA=21, SCL=22
  mpu6050.begin();
  
  Serial.println("📊 Calibrating gyroscope - keep car STILL...");
  mpu6050.calcGyroOffsets(true); // Calculate offsets with verbose output
  Serial.println("✅ MPU6500 initialized and calibrated!");
  
  // Test gyro readings
  Serial.println("🔄 Testing gyro readings...");
  for(int i = 0; i < 5; i++) {
    mpu6050.update();
    Serial.print("Angle Z: "); Serial.print(mpu6050.getAngleZ());
    Serial.print("° | Gyro Z: "); Serial.println(mpu6050.getGyroZ());
    delay(500);
  }

  // Radio Setup with detailed diagnostics
  Serial.println("📡 Initializing NRF24L01 radio...");
  if (!radio.begin()) {
    Serial.println("❌ ERROR: Radio hardware not responding!");
    Serial.println("🔧 Please check NRF24L01 wiring:");
    Serial.println("   VCC -> 3.3V, GND -> GND");
    Serial.println("   CE -> GPIO 4, CSN -> GPIO 5");
    Serial.println("   SCK -> GPIO 18, MOSI -> GPIO 23, MISO -> GPIO 19");
    while (1) {
      delay(1000);
      Serial.println("⚠️ Radio initialization failed - check wiring!");
    }
  }
  
  // Configure radio settings to match transmitter exactly
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH); // Increased power for better range and reliability
  radio.setDataRate(RF24_250KBPS); // Lower data rate for better range and reliability
  radio.setChannel(76);
  radio.setAutoAck(true); // Enable auto acknowledgments for reliability
  radio.enableAckPayload(); // Enable acknowledgment payloads
  radio.setRetries(5, 15); // More retries with longer delay for better reliability
  radio.startListening();
  
  // Print radio configuration
  Serial.println("✅ Radio initialized successfully!");
  Serial.print("📍 Channel: "); Serial.println(radio.getChannel());
  Serial.print("📊 Data Rate: "); Serial.println(radio.getDataRate());
  Serial.print("📈 PA Level: "); Serial.println(radio.getPALevel());
  Serial.print("🎯 Address: "); 
  for(int i = 0; i < 5; i++) {
    Serial.print(address[i], HEX);
  }
  Serial.println();
  
  // Enable motor driver after everything is set up
  digitalWrite(STBY, HIGH);
  Serial.println("✅ Motor driver enabled - Ready for commands!");
  
  // Show what data structure we're expecting
  Serial.println("📋 Expected data structure:");
  Serial.println("   struct ControlData { int steer; int throttle; }");
  Serial.println("   Steer: -512 to +512 (left to right)");
  Serial.println("   Throttle: -512 to +512 (reverse to forward)");
  
  Serial.println("🧭 Gyro Stabilization: DISABLED FOR TESTING");
  Serial.println("   - Gyro is temporarily disabled to test steering issues");
  Serial.println("   - Pure manual control mode");
  
  // Uncomment the line below to test gyro only
  // testGyro();
  
  Serial.println("=== 🎮 Ready! Start your transmitter and Python controller ===");
  Serial.println("=== 👀 Watching for signals... ===");
}

void loop() {
  static unsigned long lastSignalTime = 0;
  static unsigned long signalCount = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastGyroUpdate = 0;
  
  // Update gyroscope readings (every 10ms for smooth control)
  if (millis() - lastGyroUpdate > 10) {
    mpu6050.update();
    lastGyroUpdate = millis();
  }
  
  if (radio.available()) {
    // Signal detected!
    lastSignalTime = millis();
    signalCount++;
    
    // Read the data
    radio.read(&receivedData, sizeof(receivedData));
    
    // Print received data for verification with enhanced formatting (reduced frequency to prevent spam)
    if (signalCount % 20 == 0) { // Only print every 20th signal to reduce serial spam even more
      Serial.print("📡 ["); Serial.print(signalCount); Serial.print("] ");
      Serial.print("Throttle: "); Serial.print(receivedData.throttle);
      Serial.print(", Steer: "); Serial.print(receivedData.steer);
      
      // Add gyro data to display
      Serial.print(" | Gyro Z: "); Serial.print(mpu6050.getAngleZ(), 1); Serial.print("°");
    }
    
    // Validate data ranges
    bool validData = (receivedData.throttle >= -512 && receivedData.throttle <= 512 &&
                     receivedData.steer >= -512 && receivedData.steer <= 512);
    
    if (!validData) {
      if (signalCount % 20 == 0) {
        Serial.println(" ❌ INVALID DATA RANGE!");
      }
      return;
    }
      
      // --- Motor Control with detailed feedback ---
      int throttleValue = receivedData.throttle;
      String motorStatus = "";
      static int lastThrottleValue = 0; // Track previous throttle to prevent rapid switching
      static int currentSpeed = 0; // Current actual speed for gradual ramping
      static unsigned long lastSpeedUpdate = 0; // For speed ramping timing
      
      // Ensure motor driver is enabled
      digitalWrite(STBY, HIGH);
      
      // Check for direction change - use non-blocking approach
      bool directionChange = false;
      static unsigned long directionChangeTime = 0;
      static bool inDirectionChange = false;
      
      // Detect direction change
      if ((lastThrottleValue > 10 && throttleValue < -10) || 
          (lastThrottleValue < -10 && throttleValue > 10)) {
        if (!inDirectionChange) {
          directionChange = true;
          inDirectionChange = true;
          directionChangeTime = millis();
          // Brief stop before direction change to protect motor driver
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          analogWrite(PWMA, 0);
        }
      }
      
      // Check if direction change pause is complete (non-blocking)
      if (inDirectionChange && (millis() - directionChangeTime > 50)) {
        inDirectionChange = false;
      }
      
      // Only control motor if not in direction change pause
      if (!inDirectionChange) {
        int targetSpeed = 0; // Target speed based on throttle input
        
        if (throttleValue > 10) { // Forward motion
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
          targetSpeed = map(throttleValue, 10, 512, 0, 204); // Reduced max speed from 255 to 204 (20% reduction)
          
          // Gradual speed ramping for better control
          if (millis() - lastSpeedUpdate > 20) { // Update every 20ms
            if (currentSpeed < targetSpeed) {
              currentSpeed += 3; // Increase speed gradually (3 units per 20ms)
              if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            } else if (currentSpeed > targetSpeed) {
              currentSpeed -= 5; // Decrease speed faster than increase
              if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
            }
            lastSpeedUpdate = millis();
          }
          
          analogWrite(PWMA, currentSpeed);
          motorStatus = "🚗 FORWARD (Speed: " + String(currentSpeed) + "/204, Target: " + String(targetSpeed) + ")";
          if (directionChange) motorStatus += " [DIR CHANGE]";
          
        } else if (throttleValue < -10) { // Reverse motion
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          targetSpeed = map(abs(throttleValue), 10, 512, 0, 204); // Reduced max speed from 255 to 204 (20% reduction)
          
          // Gradual speed ramping for reverse too
          if (millis() - lastSpeedUpdate > 20) { // Update every 20ms
            if (currentSpeed < targetSpeed) {
              currentSpeed += 3; // Increase speed gradually
              if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            } else if (currentSpeed > targetSpeed) {
              currentSpeed -= 5; // Decrease speed faster
              if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
            }
            lastSpeedUpdate = millis();
          }
          
          analogWrite(PWMA, currentSpeed);
          motorStatus = "🔄 REVERSE (Speed: " + String(currentSpeed) + "/204, Target: " + String(targetSpeed) + ")";
          if (directionChange) motorStatus += " [DIR CHANGE]";
          
        } else { // Stop
          targetSpeed = 0;
          
          // Gradual stop for smoother braking
          if (millis() - lastSpeedUpdate > 10) { // Faster stop updates
            if (currentSpeed > 0) {
              currentSpeed -= 8; // Brake faster than acceleration
              if (currentSpeed < 0) currentSpeed = 0;
            }
            lastSpeedUpdate = millis();
          }
          
          if (currentSpeed == 0) {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
          }
          analogWrite(PWMA, currentSpeed);
          motorStatus = "⏹️ STOPPING (Speed: " + String(currentSpeed) + "/204)";
        }
      } else {
        // During direction change, force stop
        currentSpeed = 0;
        motorStatus = "⏸️ DIRECTION CHANGE PAUSE";
      }
      
      lastThrottleValue = throttleValue; // Remember last throttle value

    // --- Steering Control with Gyro Stabilization ---
    // FIXED: Reverse steering mapping so left joystick goes left, right joystick goes right
    // Map steering value (-512 to 512) to servo angle (180 to 0 degrees - REVERSED)
    
    // Add deadband for steering to prevent jitter
    int steerValue = receivedData.steer;
    if (abs(steerValue) < 20) { // Deadband of 20 units
      steerValue = 0;
    }
    
    int rawSteerAngle = map(steerValue, -512, 512, 120, 60); // Reduced steering range: 60° (right) to 120° (left), center at 90°
    
    // Gyro stabilization (only when driving mostly straight and not actively steering)
    int finalSteerAngle = rawSteerAngle;
    
    if (gyroStabilization && abs(steerValue) < 50 && abs(throttleValue) > 50) { // Only stabilize when driving forward/back and minimal steering input
      // Get current angle from gyro
      float currentAngle = mpu6050.getAngleZ();
      
      // Calculate correction needed (simple P controller)
      float angleError = targetAngle - currentAngle;
      gyroCorrection = angleError * 0.5; // Gain factor (adjust for sensitivity)
      
      // Apply correction (constrain to reasonable limits)
      gyroCorrection = constrain(gyroCorrection, -10, 10); // Reduced correction range to prevent interference
      finalSteerAngle = rawSteerAngle - (int)gyroCorrection; // Subtract for reversed mapping
      
      // Update target angle slowly toward controller input
      targetAngle = targetAngle * 0.98 + (map(steerValue, -512, 512, 15, -15)) * 0.02; // Smaller angle range for realistic steering
    } else {
      // When actively steering, update target angle
      targetAngle = map(steerValue, -512, 512, 30, -30); // Realistic steering angle range
      gyroCorrection = 0;
    }
    
    // Constrain to reduced steering range
    finalSteerAngle = constrain(finalSteerAngle, 60, 120); // 30° left and right of center
    
    steeringServo.write(finalSteerAngle);
    
    // Determine steering direction for display (updated for reduced steering range)
    String steerDirection = "";
    if (finalSteerAngle > 110) {
      steerDirection = "⬅️ FULL LEFT";
    } else if (finalSteerAngle > 100) {
      steerDirection = "⬅️ LEFT";
    } else if (finalSteerAngle > 95) {
      steerDirection = "⬅️ SLIGHT LEFT";
    } else if (finalSteerAngle < 70) {
      steerDirection = "➡️ FULL RIGHT";
    } else if (finalSteerAngle < 80) {
      steerDirection = "➡️ RIGHT";
    } else if (finalSteerAngle < 85) {
      steerDirection = "➡️ SLIGHT RIGHT";
    } else {
      steerDirection = "⬆️ STRAIGHT";
    }    // Print complete status (reduced frequency to prevent spam)
    if (signalCount % 10 == 0) {
      Serial.print(" | Motor: ");
      Serial.print(motorStatus);
      Serial.print(" | Servo: ");
      Serial.print(finalSteerAngle);
      Serial.print("° (");
      Serial.print(steerDirection);
      if (abs(gyroCorrection) > 1) {
        Serial.print(" + Gyro:");
        Serial.print(gyroCorrection, 1);
        Serial.print("°");
      }
      Serial.println(")");
    }
  } else {
    // No signal received - show status occasionally
    if (millis() - lastStatusTime > 3000) { // Every 3 seconds
      if (signalCount == 0) {
        Serial.println("⚠️ No signals received yet...");
        Serial.println("   🔧 Check: 1) Transmitter powered on?");
        Serial.println("           2) Python controller running?");
        Serial.println("           3) Both devices using same address?");
      } else {
        unsigned long timeSinceLastSignal = millis() - lastSignalTime;
        Serial.print("⚠️ No signal for "); 
        Serial.print(timeSinceLastSignal); 
        Serial.print("ms (Total received: ");
        Serial.print(signalCount);
        Serial.println(")");
        
        // Stop motor if no signal for too long
        if (timeSinceLastSignal > 2500) { // Increased from 1000ms to 2500ms to prevent false timeouts during steering
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          analogWrite(PWMA, 0);
          steeringServo.write(90); // Center steering
          Serial.println("🛑 SAFETY TIMEOUT - Motor stopped, steering centered");
        }
      }
      lastStatusTime = millis();
    }
  }
}
