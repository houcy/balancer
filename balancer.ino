// Based on Jeff Rowberg's MPU6050_DMP6.ino at
// https://github.com/jrowberg/i2cdevlib
//
// The control scheme heavily inspired by B-robot:
// https://github.com/jjrobots/B-ROBOT/blob/master/BROBOT/BROBOT.ino
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <NewPing.h>

// Pin mapping
const int BLUE_LED_PIN = 8, PING_TRIG = 4,
          PING_ECHO = 11,
          MOTOR_AA = 6,
          MOTOR_AB = 10,
          MOTOR_BA = 5,
          MOTOR_BB = 9;

// Some constants related to the control loops
// There's a PI loop for controlling the target pitch, given the target speed.
// This way the robot can be made to lean forward and accelerate.
// A PID loop then controls the motors to reach the target pitch.
const double Kpa = 0.0010, Kia = 0.0015, Kda = 0.0,  // target pitch control
             Kpm = 1200,  Kim = 5000,   Kdm = 60, // motor control
             GYRO_WEIGHT = 0.25, LOWPASS_PARAM = 0.05;
const int PID_SAMPLE_TIME = 5;

// Some motion parameters for trimming etc.
const double FALL_THRESHOLD = 0.5;
const int FALL_DELAY = 500, RAISE_DELAY = 1500;
const int THROTTLE_FWD = 140, THROTTLE_BWD = -140, THROTTLE_TURN = 120,
          TURN_LEFT = -15, TURN_RIGHT = 30, TRIM_FWD = 12, TRIM_BWD = -12;
const int PING_INTERVAL = 250, PING_RANGE = 40;
const float TARGET_PITCH_MAX = 0.22, TARGET_PITCH_MIN = -0.25;

// Some necessary state variables
int turnParam = 0;
unsigned long fallTime = 0, lastPing = 0;
bool fallen = false;
// PID loops
double targetSpeed = 0, filteredSpeed = 0, targetPitch = 0, pitch = 0,
       motorSpeed = 0;
PID pitchPID(&filteredSpeed, &targetPitch, &targetSpeed, Kpa, Kia, Kda, REVERSE);
PID motorPID(&pitch, &motorSpeed, &targetPitch, Kpm, Kim, Kdm, DIRECT);
// MPU related stuff
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint8_t mpuIntStatus;
Quaternion q;
VectorFloat gravity;
int16_t gyro[3];
// Ultrasonic sensor
NewPing sonar(PING_TRIG, PING_ECHO, PING_RANGE);

volatile bool gMpuInterrupt = false;
void mpuDataReady() {
    gMpuInterrupt = true;
}

void updateMotors() {
    updateMotorPWM(motorSpeed + turnParam, MOTOR_AA, MOTOR_AB);
    updateMotorPWM(motorSpeed - turnParam, MOTOR_BA, MOTOR_BB);
}

void updateMotorPWM(int speed, const int pinA, const int pinB) {
    if (speed < 0) {
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, constrain(255 + speed, 0, 255));
    } else {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, constrain(speed, 0, 255));
    }
}

void setup() {
    // Pin setup
    pinMode(MOTOR_AA, OUTPUT);
    pinMode(MOTOR_AB, OUTPUT);
    pinMode(MOTOR_BA, OUTPUT);
    pinMode(MOTOR_BB, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);

    updateMotors();

    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(TARGET_PITCH_MIN, TARGET_PITCH_MAX);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME);
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);
    motorPID.SetSampleTime(PID_SAMPLE_TIME);

    Wire.begin();
    Wire.setClock(400000L);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-2927);
    mpu.setYAccelOffset(1505);
    mpu.setZAccelOffset(1500);
    mpu.setXGyroOffset(105);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(31);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), mpuDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

    delay(5000);  // A delay for MPU calibration
    digitalWrite(BLUE_LED_PIN, HIGH);
}

void loop() {
    while (!gMpuInterrupt && fifoCount < packetSize) mainLoop();
    readMPU();
}

void pingCheck() {
    digitalWrite(BLUE_LED_PIN, sonar.check_timer() ? LOW : HIGH);
}

void mainLoop() {
    unsigned long currentTime = millis();
    if (!fallen) {
        if (abs(pitch) > FALL_THRESHOLD &&
            currentTime - fallTime > RAISE_DELAY) {
            motorSpeed = 0;
            targetSpeed = 0;
            turnParam = 0;
            fallTime = currentTime;
            fallen = true;
        } else {
            pitchPID.Compute();
            motorPID.Compute();
        }
    } else if (currentTime - fallTime > FALL_DELAY) fallen = false;

    if (currentTime - lastPing > PING_INTERVAL) {
        lastPing = currentTime;
        sonar.ping_timer(pingCheck);
    }

    updateMotors();
}

void readMPU() {
    gMpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO();
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        pitch = atan(gravity.x / sqrt(gravity.y*gravity.y +
                    gravity.z*gravity.z));
        double speedEstimate = motorSpeed + GYRO_WEIGHT * gyro[1];
        filteredSpeed = filteredSpeed -
            LOWPASS_PARAM * (filteredSpeed - speedEstimate);
    }
}

