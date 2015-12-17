// Based on Jeff Rowberg's MPU6050_DMP6.ino at
// https://github.com/jrowberg/i2cdevlib
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <PID_v1.h>
#include <NewPing.h>
#include <IRremote.h>

const double Kp = 900, Ki = 4000, Kd = 40;
double setPoint = 0.2, input, output;

// Pin mapping
const int IR_PIN = 7,
          BLUE_LED_PIN = 8,
          PING_TRIG = 4,
          PING_ECHO = 10,
          MOTOR_AA = 12,
          MOTOR_AB = 6,
          MOTOR_BA = 9,
          MOTOR_BB = 5;

MPU6050 mpu;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, REVERSE);
NewPing sonar(PING_TRIG, PING_ECHO, 50);
IRrecv ir(IR_PIN);
decode_results irRes;

int motorAspeed = 0,
    motorBspeed = 0,
    motorAoffset = 0,
    motorBoffset = 0;

bool ledState = LOW;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;
float ypr[3] = { 0, 0, 0 };  // [yaw, pitch, roll]

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void updateMotors() {
    updateMotorPWM(motorAspeed + motorAoffset, MOTOR_AA, MOTOR_AB);
    updateMotorPWM(motorBspeed + motorBoffset, MOTOR_BA, MOTOR_BB);
}

void updateMotorPWM(int speed, const int pinA, const int pinB) {
    if (speed < 0) {
        digitalWrite(pinA, HIGH);
        analogWrite(pinB, 255 + speed);
    } else {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, speed);
    }
}

void setup() {
    pinMode(MOTOR_AA, OUTPUT);
    pinMode(MOTOR_AB, OUTPUT);
    pinMode(MOTOR_BA, OUTPUT);
    pinMode(MOTOR_BB, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    updateMotors();
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(10);

    ir.enableIRIn();

    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
        delay(5000);
        dmpReady = true;
    }
}

void toggleLed() {
    ledState = ledState == HIGH ? LOW : HIGH;
    digitalWrite(BLUE_LED_PIN, ledState);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        input = ypr[1];
        myPID.Compute();
        motorAoffset = motorBoffset = output;
        updateMotors();

        if (ir.decode(&irRes)) {
            if (irRes.value == 0xFF02FD) // The OK button
                toggleLed();
            ir.resume();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}
