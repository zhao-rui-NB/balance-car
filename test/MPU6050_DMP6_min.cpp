
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.


MPU6050 mpu;

uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

void mpu6050_init() {
    mpu.initialize();
    Serial.println(F("Testing MPU6050 connection..."));
    if(mpu.testConnection() == false){
        Serial.println("MPU6050 connection failed");
        while(true);
    }
    else {
        Serial.println("MPU6050 connection successful");
    }

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);


    if (devStatus != 0) {
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      while(1); // Loop forever if DMP initialization fails
    } else {
      Serial.println(F("DMP Initialization successful!"));
    }


    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);


    Serial.println(F("DMP ready! Waiting for first interrupt..."));
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Wire.begin(33, 32);
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
    
    Serial.begin(115200); //115200 is required for Teapot Demo output

    mpu6050_init();
  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        //  10 ms
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        Serial.printf("passtime: %d\t ypr: %.3f\t%.3f\t%.3f\n", now - lastUpdate, ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        lastUpdate = now;
    } 
}
