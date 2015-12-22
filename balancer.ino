// Based on Jeff Rowberg's MPU6050_DMP6.ino at
// https://github.com/jrowberg/i2cdevlib
//
// The control scheme heavily inspired by B-robot:
// https://github.com/jjrobots/B-ROBOT/blob/master/BROBOT/BROBOT.ino
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
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

const double Kpa = 0.001, Kia = 0.0012, Kda = 0,
             Kpm = 1200, Kim = 5000, Kdm = 60,
             gyroWeight = 0.2, lowPass = 0.05;
double targetSpeed = 0, filteredSpeed = 0, targetPitch = 0, pitch = 0,
       motorSpeed = 0;
PID pitchPID(&filteredSpeed, &targetPitch, &targetSpeed, Kpa, Kia, Kda, DIRECT);
PID motorPID(&pitch, &motorSpeed, &targetPitch, Kpm, Kim, Kdm, REVERSE);
const int PID_SAMPLE_TIME = 10;
const float FALL_THRESHOLD = 0.5;
unsigned long fallTime = 0;
const int FALL_DELAY = 500, RAISE_DELAY = 1500;
bool fallen = false;

// Some parameters for the motor drive
int motorAspeed = 0, motorBspeed = 0;
const double THROTTLE_FORWARD = 140, THROTTLE_BACKWARD = -140,
             THROTTLE_TURN = 120;
const double TURN_LEFT = -15, TURN_RIGHT = 30, TRIM_FWD = 12, TRIM_BWD = -12;
double turnParam = 0;

MPU6050 mpu;
NewPing sonar(PING_TRIG, PING_ECHO, 50);
IRrecv ir(IR_PIN);
decode_results irRes;

// DMP helper variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
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
    updateMotorPWM(motorAspeed + turnParam, MOTOR_AA, MOTOR_AB);
    updateMotorPWM(motorBspeed - turnParam, MOTOR_BA, MOTOR_BB);
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
    pitchPID.SetOutputLimits(-0.25, 0.25);
    pitchPID.SetSampleTime(PID_SAMPLE_TIME);
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);
    motorPID.SetSampleTime(PID_SAMPLE_TIME);

    #ifdef DEBUG_ON
        Serial.begin(9600);
    #endif
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(-2946);
    mpu.setYAccelOffset(1570);
    mpu.setZAccelOffset(1482);
    mpu.setXGyroOffset(74);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(25);

    if (devStatus != 0) {
        #ifdef DEBUG_ON
            Serial.println("DMP initialization failed");
        #endif
        return;
    }

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

    delay(5000);  // A delay for MPU calibration
    dmpReady = true;
    digitalWrite(BLUE_LED_PIN, HIGH);
}

void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) mainLoop();
    readMPU();
}

void mainLoop() {
    if (!fallen) {
        if (abs(pitch) > FALL_THRESHOLD && millis() - fallTime > RAISE_DELAY) {
            motorSpeed = 0;
            targetSpeed = 0;
            turnParam = 0;
            fallTime = millis();
            fallen = true;
        } else {
            pitchPID.Compute();
            motorPID.Compute();
        }
    } else if (millis() - fallTime > FALL_DELAY) fallen = false;
    motorAspeed = motorBspeed = motorSpeed;
    updateMotors();

    if (ir.decode(&irRes)) {
        switch (irRes.value) {
            case IR_OK:
                targetSpeed = 0;
                turnParam = 0;
                break;
            case IR_UP:
                targetSpeed = THROTTLE_FORWARD;
                turnParam = TRIM_FWD;
                break;
            case IR_DOWN:
                targetSpeed = THROTTLE_BACKWARD;
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
        double speedEstimate = motorSpeed + gyroWeight * gyro[1];
        filteredSpeed = filteredSpeed -
            lowPass * (filteredSpeed - speedEstimate);
        #ifdef DEBUG_ON
            Serial.println(gyro[1]);
        #endif
    }
}
