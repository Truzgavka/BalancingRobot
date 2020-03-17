#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

Servo myservo;
MPU6050 mpu;

//65,44

bool blinkState = false;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

Quaternion q;           
VectorInt16 aa;        
VectorInt16 aaReal;    
VectorInt16 aaWorld;    
VectorFloat gravity;   
float euler[3];         
float ypr[3];  
float now;

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  mpu.initialize();
/*  while (Serial.available() && Serial.read()); ///do wywalenia
  while (!Serial.available());                //
  while (Serial.available() && Serial.read());//*/ 
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
 myservo.attach(9);
}

void loop() {
  if (!dmpReady) return;
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
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            now = ypr[1] * 180/M_PI;
            Serial.print(now);
            if(now>65.48){
              przod();
            } else if(now<65.40){
              tyl();
            }
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
  }
}

void przod(){
  myservo.write(180);
  delay(0.0001);
}

void tyl(){
  myservo.write(0);
  delay(0.0001);
}

