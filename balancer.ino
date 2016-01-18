// Based on Jeff Rowberg's MPU6050_DMP6.ino at
// https://github.com/jrowberg/i2cdevlib
//
// The control scheme heavily inspired by B-robot:
// https://github.com/jjrobots/B-ROBOT/blob/master/BROBOT/BROBOT.ino
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <NewPing.h>
#include <IRremote.h>

// IR button codes
const unsigned long IR_OK    = 0xFF02FD,
                    IR_UP    = 0xFF629D,
                    IR_DOWN  = 0xFFA857,
                    IR_LEFT  = 0xFF22DD,
                    IR_RIGHT = 0xFFC23D,
                    IR_REP   = 0xFFFFFFFF;

// Pin mapping
const int IR_PIN = 7,
          BLUE_LED_PIN = 8,
          PING_TRIG = 4,
          PING_ECHO = 10,
          MOTOR_AA = 12,
          MOTOR_AB = 6,
          MOTOR_BA = 9,
          MOTOR_BB = 5;

// Some constants related to the control loops
// There's a PI loop for controlling the target pitch, given the target speed.
// This way the robot can be made to lean forward and accelerate.
// A PID loop then controls the motors to reach the target pitch.
const double Kpa = 0.001, Kia = 0.0012, Kda = 0,  // target pitch control
             Kpm = 1200,  Kim = 5000,   Kdm = 60, // motor control
             GYRO_WEIGHT = 0.2, LOWPASS_PARAM = 0.05;
const int PID_SAMPLE_TIME = 10;

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
PID pitchPID(&filteredSpeed, &targetPitch, &targetSpeed, Kpa, Kia, Kda, DIRECT);
PID motorPID(&pitch, &motorSpeed, &targetPitch, Kpm, Kim, Kdm, REVERSE);
// Object for the sensors etc.
MPU6050 mpu;
NewPing sonar(PING_TRIG, PING_ECHO, PING_RANGE);
IRrecv ir(IR_PIN);
decode_results irRes;

// DMP helper variables
uint8_t mpuIntStatus;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;
int16_t gyro[3] = { 0, 0, 0 };

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
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
    // A little initial delay to get the robot's lid closed
    delay(4000);
    // Pin setup
    pinMode(MOTOR_AA, OUTPUT);
    pinMode(MOTOR_AB, OUTPUT);
    pinMode(MOTOR_BA, OUTPUT);
    pinMode(MOTOR_BB, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);

    updateMotors();
    ir.enableIRIn();

    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(TARGET_PITCH_MIN, TARGET_PITCH_MAX);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME);
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);
    motorPID.SetSampleTime(PID_SAMPLE_TIME);

    #ifdef DEBUG_ON
        Serial.begin(9600);
    #endif
    Fastwire::setup(400, true);
    mpu.initialize();

    if (mpu.dmpInitialize() != 0) {
        #ifdef DEBUG_ON
            Serial.println("DMP initialization failed");
        #endif
        return;
    }

    mpu.setXAccelOffset(-2946);
    mpu.setYAccelOffset(1570);
    mpu.setZAccelOffset(1482);
    mpu.setXGyroOffset(74);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(25);

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

    delay(5000);  // A delay for MPU calibration
    digitalWrite(BLUE_LED_PIN, HIGH);
}

void loop() {
    while (!mpuInterrupt && fifoCount < packetSize) mainLoop();
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

    if (ir.decode(&irRes)) {
        #ifdef DEBUG_ON
            Serial.println(irRes.value, HEX);
        #endif
        switch (irRes.value) {
            case IR_OK:
                targetSpeed = 0;
                turnParam = 0;
                break;
            case IR_UP:
                targetSpeed = THROTTLE_FWD;
                turnParam = TRIM_FWD;
                break;
            case IR_DOWN:
                targetSpeed = THROTTLE_BWD;
                turnParam = TRIM_BWD;
                break;
            case IR_LEFT:
                turnParam = TURN_LEFT;
                targetSpeed = THROTTLE_TURN;
                break;
            case IR_RIGHT:
                turnParam = TURN_RIGHT;
                targetSpeed = THROTTLE_TURN;
                break;
        }
        ir.resume();
    }

    updateMotors();
}

void readMPU() {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        // We only need the pitch - hence no call to getYawPitchRoll
        pitch = atan(gravity.x / sqrt(gravity.y*gravity.y +
            gravity.z*gravity.z));
        double speedEstimate = motorSpeed + GYRO_WEIGHT * gyro[1];
        filteredSpeed = filteredSpeed -
            LOWPASS_PARAM * (filteredSpeed - speedEstimate);
        #ifdef DEBUG_ON
            Serial.println(gyro[1]);
        #endif
    }
}
