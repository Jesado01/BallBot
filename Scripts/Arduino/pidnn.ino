#include <digitalWriteFast.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#include <PID_v1.h>

// Serial input buffer and variables
const byte bufferSize = 32;
char buffer[bufferSize];
float pidnn_pitch = 0;
float pidnn_roll = 0;
byte index = 0;

MPU6050 mpu;

// MPU interrupt pin
#define interruptPin 2

// MPU6050 DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];  // Yaw, Pitch, Roll angles

// Encoder pins
int enc1 = 3;
int enc2 = 18;
int enc3 = 19;

// Motor control pins
int pwm1 = 6, motA1 = 5, motB1 = 4;
int pwm2 = 9, motA2 = 7, motB2 = 8;
int pwm3 = 12, motA3 = 10, motB3 = 11;

// Encoder counters
volatile int contador1 = 0;
volatile int contador2 = 0;
volatile int contador3 = 0;

// Timing
unsigned long previousMillis = 0;
long interval = 10;
float elapsedTime, time, timePrev;

// Orientation
float pitch, roll;
float pitchOffset = 0, rollOffset = 0;

// MPU interrupt handler
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// MPU offset calibration
void calibrateMPUOffsets() {
  int samples = 200;
  float pitchSum = 0, rollSum = 0;

  for (int i = 0; i < samples; i++) {
    readMPU();
    pitchSum += pitch;
    rollSum += roll;
    delay(10);
  }

  pitchOffset = pitchSum / samples;
  rollOffset = rollSum / samples;
}

// Arduino setup
void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #endif

  mpu.initialize();
  pinMode(interruptPin, INPUT);
  devStatus = mpu.dmpInitialize();

  // MPU custom offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // Motor and encoder setup
  pinMode(motA1, OUTPUT); pinMode(motB1, OUTPUT); pinMode(pwm1, OUTPUT);
  pinMode(motA2, OUTPUT); pinMode(motB2, OUTPUT); pinMode(pwm2, OUTPUT);
  pinMode(motA3, OUTPUT); pinMode(motB3, OUTPUT); pinMode(pwm3, OUTPUT);
  pinMode(13, OUTPUT);

  Serial.begin(2000000);
  attachInterrupt(digitalPinToInterrupt(enc1), inter1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2), inter2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc3), inter3, RISING);
  digitalWrite(13, HIGH);

  calibrateMPUOffsets();
}

// Main loop
void loop() {
  unsigned long currentMillis = millis();

  // Run control every interval
  if ((currentMillis - previousMillis) >= interval) {
    readMPU();
    Serial.print(pitch);
    Serial.print("@");
    Serial.println(roll);
    previousMillis = currentMillis;
  }

  // Process incoming serial data
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();

    if (incomingByte == '\n') {
      buffer[index] = '\0';

      int atIndex = -1;
      for (byte i = 0; i < index; i++) {
        if (buffer[i] == '@') {
          atIndex = i;
          break;
        }
      }

      if (atIndex != -1) {
        buffer[atIndex] = '\0';
        pidnn_pitch = atof(buffer);
        pidnn_roll = atof(buffer + atIndex + 1);
        setMotorSpeed(pidnn_pitch, pidnn_roll);
      }

      index = 0;
    } else if (index < bufferSize - 1) {
      buffer[index++] = incomingByte;
    }
  }

  // Reset encoder counters
  contador1 = 0;
  contador2 = 0;
  contador3 = 0;
}

// Interrupt service routines for encoders
void inter1() { contador1++; }
void inter2() { contador2++; }
void inter3() { contador3++; }

// Read orientation from MPU6050
void readMPU() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
  }
}

// Apply control signal to motors
void setMotorSpeed(long pitchOutput, long rollOutput) {
  float m1 = -0.5 * pitchOutput + 0.866 * rollOutput;
  float m2 = 1.0 * rollOutput;
  float m3 = -0.5 * pitchOutput - 0.866 * rollOutput;

  float scaleFactor = 1;
  m1 *= scaleFactor;
  m2 *= scaleFactor;
  m3 *= scaleFactor;

  int pwm1_calc = constrain(m1, -255, 255);
  int pwm2_calc = constrain(m2, -255, 255);
  int pwm3_calc = constrain(m3, -255, 255);

  controlMotor(pwm1_calc, motA1, motB1, pwm1);
  controlMotor(pwm2_calc, motA2, motB2, pwm2);
  controlMotor(pwm3_calc, motA3, motB3, pwm3);
}

// Control a single motor using H-bridge
void controlMotor(int pwmValue, int motorA, int motorB, int pwmPin) {
  if (pwmValue > 0) {
    digitalWriteFast(motorA, HIGH);
    digitalWriteFast(motorB, LOW);
  } else if (pwmValue < 0) {
    digitalWriteFast(motorA, LOW);
    digitalWriteFast(motorB, HIGH);
  } else {
    digitalWriteFast(motorA, LOW);
    digitalWriteFast(motorB, LOW);
  }
  analogWrite(pwmPin, abs(pwmValue));
}
