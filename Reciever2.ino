#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- MOTOR SPEED PINS (PWM) ---
int motorFR = 3; int motorFL = 5; 
int motorBR = 6; int motorBL = 9; 

// --- DIRECTION PINS (Your Specific Wiring) ---
// Driver 1
int in1 = A0; int in2 = A1; 
int in3 = A2; int in4 = A3;
// Driver 2 (Modified to avoid D7/D8 conflict with Radio)
int in5 = 2;  int in6 = 4;
int in7 = 10; int in8 = A6; 

RF24 radio(7, 8); // CE on 7, CSN on 8
const byte address[6] = "Nyx1";
MPU6050 mpu(Wire);

struct Data_Package {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};
Data_Package data;

// PID TUNING
float p_gain = 1.3; float d_gain = 15.0; 
float pitch_error, roll_error, pitch_last_error, roll_last_error, pitch_pid, roll_pid;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // 1. MPU CALIBRATION
  byte status = mpu.begin();
  Serial.println(F("CALIBRATING... KEEP DRONE LEVEL AND STILL"));
  delay(2000); 
  mpu.calcOffsets(); 
  
  // 2. RADIO SETUP
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  // 3. PIN MODES FOR SPEED
  pinMode(motorFR, OUTPUT); pinMode(motorFL, OUTPUT);
  pinMode(motorBR, OUTPUT); pinMode(motorBL, OUTPUT);
  
  // 4. PIN MODES FOR DIRECTION
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT); pinMode(in6, OUTPUT); 
  pinMode(in7, OUTPUT); pinMode(in8, OUTPUT);

  // 5. LOCK DIRECTIONS TO FORWARD
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH); digitalWrite(in6, LOW);
  digitalWrite(in7, HIGH); digitalWrite(in8, LOW);
}

void loop() {
  // A. Check for Radio Signal
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
  }

  // B. Update Sensor Data
  mpu.update();

  // C. Calculate Tilt Targets from Joysticks
  float target_pitch = map(data.pitch, 0, 255, -20, 20);
  float target_roll  = map(data.roll, 0, 255, -20, 20);

  // D. PID Calculations
  pitch_error = mpu.getAngleY() - target_pitch;
  pitch_pid = (p_gain * pitch_error) + (d_gain * (pitch_error - pitch_last_error));
  pitch_last_error = pitch_error;

  roll_error = mpu.getAngleX() - target_roll;
  roll_pid = (p_gain * roll_error) + (d_gain * (roll_error - roll_last_error));
  roll_last_error = roll_error;

  // E. Motor Mixing & Safety Check
  if (data.throttle < 15) { 
    stopMotors(); 
  } else {
    // Front motors correct for Pitch (Pitch PID is subtracted)
    // Left/Right motors correct for Roll
    analogWrite(motorFR, constrain(data.throttle - pitch_pid + roll_pid, 0, 255));
    analogWrite(motorFL, constrain(data.throttle - pitch_pid - roll_pid, 0, 255));
    analogWrite(motorBR, constrain(data.throttle + pitch_pid + roll_pid, 0, 255));
    analogWrite(motorBL, constrain(data.throttle + pitch_pid - roll_pid, 0, 255));
  }
}

// EMERGENCY BRAKE FUNCTION
void stopMotors() {
  analogWrite(motorFR, 0); analogWrite(motorFL, 0);
  analogWrite(motorBR, 0); analogWrite(motorBL, 0);
}
