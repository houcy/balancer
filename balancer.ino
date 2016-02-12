// The control scheme heavily inspired by B-robot:
// https://github.com/jjrobots/B-ROBOT/blob/master/BROBOT/BROBOT.ino
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <NewPing.h>

// Pin mapping
const int BLUE_LED_PIN = 8, PING_TRIG = 4, PING_ECHO = 11,
          MOTOR_AA = 6, MOTOR_AB = 10,
          MOTOR_BA = 5, MOTOR_BB = 9;

// Some constants related to the control loops
// There's a PI loop for controlling the target pitch, given the target speed.
// This way the robot can be made to lean forward and accelerate.
// A PID loop then controls the motors to reach the target pitch.
const double Kpa = 0.0010, Kia = 0.0012, Kda = 0,  // target pitch control
             Kpm = 1200,   Kim = 5000,   Kdm = 60, // motor control
             GYRO_WEIGHT = 0.25, LOWPASS_PARAM = 0.05;
const int PID_SAMPLE_TIME = 5;

// Some motion parameters for trimming etc.
const double FALL_THRESHOLD = 0.5;
const int FALL_DELAY = 500, RAISE_DELAY = 1500,
          REVERSE_TIME = 1000, START_DELAY = 6000;
const int THROTTLE_FWD = -105, THROTTLE_BWD = 80,
          TRIM_LEFT = -35, TRIM_RIGHT = 25, TRIM_FWD = 6;
const int PING_INTERVAL = 100, PING_RANGE = 15;
const float TARGET_PITCH_MAX = 0.22, TARGET_PITCH_MIN = -0.2,
            TARGET_PITCH_BALANCE = 0.15;

// Some necessary state variables
unsigned long reverseTime = 0, fallTime = 0, lastPing = 0;
int turnParam = 0;
bool fallen = true, stopped = false;
// PID loops
double filteredSpeed = 0, targetPitch = 0, pitch = 0, motorSpeed = 0,
       targetSpeed = 0;
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

void updateMotors() {
    updateMotorPWM(stopped ? 0 : motorSpeed + turnParam, MOTOR_AA, MOTOR_AB);
    updateMotorPWM(stopped ? 0 : motorSpeed - turnParam, MOTOR_BA, MOTOR_BB);
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

volatile bool gMpuInterrupt = false;
void mpuDataReady() {
    gMpuInterrupt = true;
}

void setup() {
    // Pin setup
    pinMode(MOTOR_AA, OUTPUT);
    pinMode(MOTOR_AB, OUTPUT);
    pinMode(MOTOR_BA, OUTPUT);
    pinMode(MOTOR_BB, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);

    pitchPID.SetOutputLimits(TARGET_PITCH_MIN, TARGET_PITCH_MAX);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME);
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

    stop();
    updateMotors();

    delay(START_DELAY);
}

void loop() {
    while (!gMpuInterrupt && fifoCount < packetSize) mainLoop();
    readMPU();
}

void forward() {
    targetSpeed = THROTTLE_FWD;
    turnParam = TRIM_FWD;
    digitalWrite(BLUE_LED_PIN, HIGH);
    stopped = false;
    reverseTime = 0;
}

void reverseAndTurn() {
    if (random(0, 2) == 0) turnParam = TRIM_LEFT;
    else turnParam = TRIM_RIGHT;
    targetSpeed = THROTTLE_BWD;
    stopped = false;
    digitalWrite(BLUE_LED_PIN, LOW);
}

void stop() {
    sonar.timer_stop();
    motorSpeed = 0;
    targetSpeed = 0;
    targetPitch = TARGET_PITCH_BALANCE;
    turnParam = 0;
    reverseTime = 0;
    stopped = true;
    digitalWrite(BLUE_LED_PIN, LOW);
}

volatile bool gObstacleDetected = false;
void pingCheck() {
    if (sonar.check_timer()) gObstacleDetected = true;
}

void mainLoop() {
    unsigned long currentTime = millis();

    if (!fallen) {
        if (abs(pitch) > FALL_THRESHOLD &&
            currentTime - fallTime > RAISE_DELAY) {
            // Oh no, we've fallen again!
            pitchPID.SetMode(MANUAL);
            motorPID.SetMode(MANUAL);
            stop();
            fallTime = currentTime;
            fallen = true;
        } else {
            // Normal operation when not fallen or trying to raise up
            if (gObstacleDetected) {
                // If obstacle detected, start reversing
                gObstacleDetected = false;
                reverseTime = currentTime;
                reverseAndTurn();
            }

            if (reverseTime != 0 && currentTime - reverseTime > REVERSE_TIME) {
                // We should revert from reversing
                forward();
            } else if (currentTime - lastPing > PING_INTERVAL) {
                lastPing = currentTime;
                sonar.ping_timer(pingCheck);
            }
        }
    } else if (currentTime - fallTime > FALL_DELAY) {
        fallen = false;
        forward();
        pitchPID.SetMode(AUTOMATIC);
        motorPID.SetMode(AUTOMATIC);
    }

    pitchPID.Compute();
    motorPID.Compute();
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

        pitch = atan(gravity.x /
            sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
        double speedEstimate = motorSpeed + GYRO_WEIGHT * gyro[1];
        filteredSpeed = filteredSpeed -
            LOWPASS_PARAM * (filteredSpeed - speedEstimate);

        #ifdef DEBUG
            Serial.println(gyro[1]);
        #endif
    }
}

