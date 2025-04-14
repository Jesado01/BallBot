#include <digitalWriteFast.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>

// MPU6050 and DMP initialization
MPU6050 mpu;
#define interruptPin 2
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll

// Motor encoder inputs
int enc1 = 3;
int enc2 = 18;
int enc3 = 19;

// Motor control pins
int pwm1 = 6, motA1 = 5, motB1 = 4;
int pwm2 = 9, motA2 = 7, motB2 = 8;
int pwm3 = 12, motA3 = 10, motB3 = 11;

// Velocity measurement and encoder counters
volatile int contador1 = 0;
volatile int contador2 = 0;
volatile int contador3 = 0;

// PID variables
float pitch, roll;
float pitchOffset = 0, rollOffset = 0;
float elapsedTime, time, timePrev;
float pid_pitch, pid_roll;
float previous_error_pitch, previous_error_roll;
float pid_p_pitch = 0, pid_i_pitch = 0, pid_d_pitch = 0;
float pid_p_roll = 0, pid_i_roll = 0, pid_d_roll = 0;
float kp = 30, ki = 0.3, kd = 0.05;
float desired_angle = 0;
float smoothed_pitch, smoothed_roll;

// Interrupt flag
volatile bool mpuInterrupt = false;

// MPU6050 interrupt callback
void dmpDataReady() {
  mpuInterrupt = true;
}

// Calculate MPU offsets for pitch and roll
void calibrarOffsetMPU() {
  int numLecturas = 200;
  float pitchSum = 0, rollSum = 0;

  for (int i = 0; i < numLecturas; i++) {
    MPU6050();  // read pitch and roll
    pitchSum += pitch;
    rollSum += roll;
    delay(10);
  }

  pitchOffset = pitchSum / numLecturas;
  rollOffset = rollSum / numLecturas;
}

// Setup all system components
void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // Set I2C to 400kHz
  #endif

  mpu.initialize();
  pinMode(interruptPin, INPUT);
  devStatus = mpu.dmpInitialize();

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

  // Motor pin configuration
  pinMode(motA1, OUTPUT); pinMode(motB1, OUTPUT); pinMode(pwm1, OUTPUT);
  pinMode(motA2, OUTPUT); pinMode(motB2, OUTPUT); pinMode(pwm2, OUTPUT);
  pinMode(motA3, OUTPUT); pinMode(motB3, OUTPUT); pinMode(pwm3, OUTPUT);
  pinMode(13, OUTPUT);

  // Serial and encoder setup
  Serial.begin(2000000);
  attachInterrupt(digitalPinToInterrupt(enc1), inter1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2), inter2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc3), inter3, RISING);
  digitalWrite(13, HIGH);

  calibrarOffsetMPU(); // Calibrate pitch/roll offsets
}

// Main control loop
void loop() {
  float pv1 = (60 * contador1 / 11.0) * 10;
  float pv2 = (60 * contador2 / 11.0) * 10;
  float pv3 = (60 * contador3 / 11.0) * 10;

  MP6050();  // update pitch, roll and compute PID

  Serial.print(pitch);
  Serial.print("@");
  Serial.println(roll);

  setMotorSpeed(pid_pitch, pid_roll);

  // Reset encoder counters
  contador1 = 0;
  contador2 = 0;
  contador3 = 0;
}

// Encoder interrupts
void inter1() { contador1++; }
void inter2() { contador2++; }
void inter3() { contador3++; }

// Read orientation and compute PID
void MP6050() {
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

  // Smoothing values
  float alpha = 1;
  smoothed_pitch = alpha * pitch;
  smoothed_roll = alpha * roll;

  // Calculate errors
  float error_pitch = pitch - desired_angle;
  float error_roll = roll - desired_angle;

  // PID for pitch
  pid_p_pitch = kp * error_pitch;
  pid_i_pitch += ki * error_pitch * elapsedTime;
  pid_d_pitch = kd * (error_pitch - previous_error_pitch) / elapsedTime;
  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;

  // PID for roll
  pid_p_roll = kp * error_roll;
  pid_i_roll += ki * error_roll * elapsedTime;
  pid_d_roll = kd * (error_roll - previous_error_roll) / elapsedTime;
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;

  previous_error_pitch = error_pitch;
  previous_error_roll = error_roll;

  mpuInterrupt = false;
}

// Apply PID output to each motor
void setMotorSpeed(long pitchOutput, long rollOutput) {
  float m1 = -0.5 * rollOutput + 0.866 * pitchOutput;
  float m2 = 1.0 * pitchOutput;
  float m3 = -0.5 * rollOutput - 0.866 * pitchOutput;

  m1 *= 1;
  m2 *= 1;
  m3 *= 1;

  int pwm1_calc = constrain(m1, -255, 255);
  int pwm2_calc = constrain(m2, -255, 255);
  int pwm3_calc = constrain(m3, -255, 255);

  controlMotor(pwm1_calc, motA1, motB1, pwm1);
  controlMotor(pwm2_calc, motA2, motB2, pwm2);
  controlMotor(pwm3_calc, motA3, motB3, pwm3);

  Serial.print(pwm1_calc); Serial.print("@");
  Serial.print(pwm2_calc); Serial.print("@");
  Serial.print(pwm3_calc); Serial.print("@");
}

// Set direction and speed for a single motor
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
